using System.Diagnostics;
using System.Text;
using System.Text.Json;

namespace Tiller;

public sealed class WorkerFailure(string message, bool writing, bool retryable = false) : Exception(message)
{
    public bool Writing { get; } = writing;
    public bool Retryable { get; } = retryable;
}

/// <summary>Owns one process tree; continuously drains both pipes without blocking the UI.</summary>
public sealed class WorkerClient
{
    private volatile bool writing;
    public bool Writing { get => writing; private set => writing = value; }
    public TimeSpan IdleLimit { get; init; } = TimeSpan.FromSeconds(45);
    public TimeSpan TotalLimit { get; init; } = TimeSpan.FromHours(2);
    public async Task RunAsync(string executable, object request, StreamWriter log,
        Action<JsonElement> onEvent, Func<Task<bool>> confirm, CancellationToken token,
        IEnumerable<string>? arguments = null)
    {
        Writing = false;
        var info = new ProcessStartInfo(executable)
        {
            UseShellExecute = false, CreateNoWindow = true,
            RedirectStandardInput = true, RedirectStandardOutput = true, RedirectStandardError = true,
            StandardOutputEncoding = Encoding.UTF8, StandardErrorEncoding = Encoding.UTF8,
            StandardInputEncoding = new UTF8Encoding(false), WorkingDirectory = Path.GetDirectoryName(executable)!
        };
        if (arguments != null) foreach (var argument in arguments) info.ArgumentList.Add(argument);
        using var process = new Process { StartInfo = info };
        var clock = Stopwatch.StartNew();
        long lastActivity = 0;
        var waiting = 0;
        var terminal = false;
        string phase = "";
        long maxProgress = -1;
        WorkerFailure? failure = null;
        var logLock = new object();
        void Log(string text) { lock (logLock) { log.Write(text); log.Flush(); } }
        void Kill()
        {
            try { if (!process.HasExited) process.Kill(entireProcessTree: true); } catch (InvalidOperationException) { }
        }
        async Task DrainErrors()
        {
            var buffer = new char[4096];
            int count;
            while ((count = await process.StandardError.ReadAsync(buffer)) > 0)
                Log(new string(buffer, 0, count));
        }
        async Task ReadEvents()
        {
            // Protocol emits short lines; bound malformed output instead of buffering indefinitely.
            var buffer = new char[4096];
            var pending = new StringBuilder();
            int count;
            while ((count = await process.StandardOutput.ReadAsync(buffer)) > 0)
            {
                for (var i = 0; i < count; i++)
                {
                    if (buffer[i] != '\n')
                    {
                        if (pending.Length >= 65536) throw new InvalidDataException("Слишком длинное сообщение рабочего процесса.");
                        pending.Append(buffer[i]); continue;
                    }
                    var line = pending.ToString(); pending.Clear();
                    Log(line + Environment.NewLine);
                    using var doc = JsonDocument.Parse(line);
                    var data = doc.RootElement.Clone();
                    if (data.GetProperty("v").GetInt32() != 1) throw new InvalidDataException("Несовместимая версия протокола.");
                    var type = data.GetProperty("type").GetString();
                    if (type == "progress")
                    {
                        var currentPhase = data.GetProperty("phase").GetString() ?? "";
                        var value = data.GetProperty("current").GetInt64();
                        // Repeated percentage or log chatter does not conceal a stalled transfer.
                        if (currentPhase != phase || value > maxProgress || (currentPhase == "transfer" && value < maxProgress))
                        { phase = currentPhase; maxProgress = value; Interlocked.Exchange(ref lastActivity, clock.ElapsedMilliseconds); }
                    }
                    else Interlocked.Exchange(ref lastActivity, clock.ElapsedMilliseconds);
                    if (type == "writing") Writing = true;
                    if (type == "done" || type == "cancelled") terminal = true;
                    if (type == "error") failure = new WorkerFailure(data.GetProperty("message").GetString() ?? "Ошибка устройства", Writing || data.GetProperty("writing").GetBoolean(), data.GetProperty("retryable").GetBoolean());
                    onEvent(data);
                    if (type == "ready")
                    {
                        Volatile.Write(ref waiting, 1);
                        clock.Stop(); // Human decision time is not transport time.
                        bool approved;
                        try { approved = await confirm().WaitAsync(token); }
                        finally { clock.Start(); Interlocked.Exchange(ref lastActivity, clock.ElapsedMilliseconds); Volatile.Write(ref waiting, 0); }
                        token.ThrowIfCancellationRequested();
                        // Disable cancellation/closing before sending permission, not after the worker starts writing.
                        if (approved) { Writing = true; onEvent(JsonSerializer.SerializeToElement(new { v = 1, type = "write_authorized" })); }
                        await process.StandardInput.WriteLineAsync(approved ? "WRITE" : "CANCEL");
                        await process.StandardInput.FlushAsync();
                    }
                }
            }
            if (pending.Length != 0) throw new InvalidDataException("Рабочий процесс оборвал сообщение.");
        }
        Task? stdout = null, stderr = null;
        try
        {
            if (!process.Start()) throw new IOException("Не удалось запустить рабочий процесс.");
            using var cancellation = token.Register(Kill);
            stdout = ReadEvents(); stderr = DrainErrors();
            await process.StandardInput.WriteLineAsync(JsonSerializer.Serialize(request));
            await process.StandardInput.FlushAsync();
            var exited = process.WaitForExitAsync();
            while (!exited.IsCompleted || !stdout.IsCompleted || !stderr.IsCompleted)
            {
                token.ThrowIfCancellationRequested();
                if (stdout.IsFaulted) await stdout;
                if (stderr.IsFaulted) await stderr;
                if (Volatile.Read(ref waiting) == 0 &&
                    (clock.ElapsedMilliseconds - Interlocked.Read(ref lastActivity) > IdleLimit.TotalMilliseconds || clock.Elapsed > TotalLimit))
                    throw new WorkerFailure("Нет продвижения операции. Связь остановлена; лог сохранён.", Writing, !Writing);
                await Task.Delay(100, token);
            }
            await Task.WhenAll(stdout, stderr, exited);
            if (failure != null) throw failure;
            if (process.ExitCode != 0 || !terminal)
                throw new WorkerFailure($"Рабочий процесс завершился без подтверждения результата (код {process.ExitCode}).", Writing);
        }
        catch (OperationCanceledException) { try { Log($"\nCANCELLED; write authorized={Writing}\n"); } catch { } throw; }
        catch (WorkerFailure error) { try { Log($"\nFAILED: {error.Message}; write authorized={Writing}\n"); } catch { } throw; }
        catch (Exception error) { try { Log($"\nFAILED: {error.Message}; write authorized={Writing}\n"); } catch { } throw new WorkerFailure(error.Message, Writing); }
        finally
        {
            Kill();
            // Keep log alive until both readers stop. Never leave an esptool/PyInstaller child behind.
            if (stdout != null && stderr != null)
            {
                try { await Task.WhenAll(stdout, stderr).WaitAsync(TimeSpan.FromSeconds(5)); } catch { }
            }
        }
    }
}
