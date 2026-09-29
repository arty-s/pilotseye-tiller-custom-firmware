using System.Diagnostics;
using System.Text.Json;
using Tiller;

if (args.Length > 0)
{
    Console.ReadLine();
    void Emit(object data) { Console.WriteLine(JsonSerializer.Serialize(data)); Console.Out.Flush(); }
    switch (args[0])
    {
        case "good":
            for (var i = 0; i < 5; i++) { Emit(new { v = 1, type = "progress", phase = "backup", current = i, total = 5 }); await Task.Delay(60); }
            Emit(new { v = 1, type = "done", message = "ok" }); break;
        case "flood":
            await Console.Error.WriteAsync(new string('x', 200000)); Emit(new { v = 1, type = "done" }); break;
        case "stall":
            for (var i = 0; i < 100; i++) { Emit(new { v = 1, type = "progress", phase = "backup", current = 4, total = 100 }); await Task.Delay(100); } break;
        case "silent": await Task.Delay(15000); break;
        case "crash": return 7;
        case "partial": Console.Write("{\"v\":"); break;
        case "oversize": Console.Write(new string('x', 100000)); await Task.Delay(5000); break;
        case "ready":
            Emit(new { v = 1, type = "ready" });
            var reply = Console.ReadLine();
            if (reply == "WRITE") { Emit(new { v = 1, type = "writing" }); Emit(new { v = 1, type = "done" }); }
            else Emit(new { v = 1, type = "cancelled" });
            break;
        case "write-crash": Emit(new { v = 1, type = "writing" }); return 8;
    }
    return 0;
}

var executable = Environment.ProcessPath!;
var temp = Path.Combine(Path.GetTempPath(), "TillerRunnerTests-" + Guid.NewGuid().ToString("N"));
Directory.CreateDirectory(temp);
var count = 0;
async Task Test(string mode, bool shouldFail, bool consent = false, bool expectWrite = false, int cancelAfter = 0)
{
    var client = new WorkerClient { IdleLimit = TimeSpan.FromMilliseconds(800), TotalLimit = mode == "ready" ? TimeSpan.FromMilliseconds(800) : TimeSpan.FromSeconds(8) };
    using var cancel = new CancellationTokenSource();
    if (cancelAfter > 0) cancel.CancelAfter(cancelAfter);
    await using var log = new StreamWriter(Path.Combine(temp, mode + ".log"));
    var stopwatch = Stopwatch.StartNew();
    var events = 0;
    Exception? error = null;
    try
    {
        await client.RunAsync(executable, new { action = "test" }, log, _ => events++,
            async () => { await Task.Delay(1000); return consent; }, cancel.Token, [mode]);
    }
    catch (Exception e) { error = e; }
    if ((error != null) != shouldFail) throw new Exception($"{mode}: wrong result: {error}");
    if (client.Writing != expectWrite) throw new Exception(mode + ": wrong write state");
    if (stopwatch.Elapsed.TotalSeconds > 5) throw new Exception(mode + ": runner hung");
    if (!shouldFail && events == 0) throw new Exception(mode + ": no events");
    Console.WriteLine($"PASS {mode}, {stopwatch.ElapsedMilliseconds} ms, events={events}"); count++;
}
await Test("good", false);
await Test("flood", false);
await Test("stall", true);
await Test("silent", true);
await Test("crash", true);
await Test("partial", true);
await Test("oversize", true);
await Test("ready", false, consent: true, expectWrite: true);
await Test("ready", false);
await Test("ready", true, cancelAfter: 300);
await Test("silent", true, cancelAfter: 300);
await Test("write-crash", true, expectWrite: true);
var discovered = DeviceDiscovery.PreferTillerPorts([
    ("COM12", true),
    ("COM5", false),
    ("COM9", false)
]);
if (discovered.Count != 1 || discovered[0] != "COM12")
    throw new Exception("port discovery: one real Tiller was reported as multiple devices: " + string.Join(", ", discovered));
Console.WriteLine("PASS port discovery prefers VID_031F:PID_9001"); count++;
Console.WriteLine($"All {count} process tests passed. Logs: {temp}");
return 0;
