using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform.Storage;
using Avalonia.Threading;
using Microsoft.Win32;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;

namespace Tiller;

public sealed class MainWindow : Window
{
    public static string DataRoot => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Tiller");
    private static string Backups => Path.Combine(DataRoot, "backups");
    private static readonly IBrush Canvas = Brush.Parse("#091018"), Surface = Brush.Parse("#0E1720"), Field = Brush.Parse("#071018");
    private static readonly IBrush Ink = Brush.Parse("#EBF2F7"), Muted = Brush.Parse("#8CA3B5"), Accent = Brush.Parse("#33C9E8");
    private static readonly IBrush Success = Brush.Parse("#40D792"), Line = Brush.Parse("#293847");

    private readonly TextBlock portStatus = Text("Тиллер не найден", 12, true);
    private readonly TextBlock state = Text("Ожидание устройства", 12);
    private readonly TextBlock percent = Text("0%", 12, true);
    private readonly TextBlock firmwareName = Text("piloteye_tiller.ino.bin", 14);
    private readonly TextBlock details = Text("Подключите тиллер по USB.", 14);
    private readonly CheckBox makeBackup = new() { Content = "Создать резервную копию", IsChecked = true, FontSize = 14 };
    private readonly ProgressBar progress = new() { Minimum = 0, Maximum = 100, Height = 8, Foreground = Accent, Background = Brush.Parse("#18232D") };
    private readonly Button chooseFirmware = Button("Выбрать…"), update = Button("Обновить");
    private readonly DispatcherTimer portTimer = new() { Interval = TimeSpan.FromSeconds(1) };
    private WorkerClient worker = new();
    private CancellationTokenSource? cancellation;
    private string firmwarePath = "", selectedPort = "", logPath = "";
    private bool busy, fatal;

    public MainWindow()
    {
        Title = "Tiller · обновление прошивки";
        Width = 720; Height = 560; MinWidth = 640; MinHeight = 520;
        Background = Canvas; Foreground = Ink; FontFamily = new("Segoe UI Variable, Segoe UI"); FontSize = 14;
        Directory.CreateDirectory(Backups); Directory.CreateDirectory(Path.Combine(DataRoot, "logs"));
        firmwarePath = Path.Combine(AppContext.BaseDirectory, "package", "firmware", "app.bin");

        update.Background = Accent; update.Foreground = Brush.Parse("#031018"); update.FontWeight = FontWeight.SemiBold;
        update.HorizontalAlignment = HorizontalAlignment.Stretch; update.HorizontalContentAlignment = HorizontalAlignment.Center;

        var header = new Grid { ColumnDefinitions = new("*,Auto"), Height = 72, Background = Surface };
        var brand = new StackPanel { Spacing = 0, Margin = new(28, 15) };
        brand.Children.Add(Text("Tiller", 14, true)); brand.Children.Add(Text("Обновление прошивки", 12)); header.Children.Add(brand);
        var portPill = new Border { Child = portStatus, Background = Brush.Parse("#0C2A24"), BorderBrush = Brush.Parse("#24664F"), BorderThickness = new(1), CornerRadius = new(18), Padding = new(12, 7), Margin = new(0, 16, 28, 16) };
        Grid.SetColumn(portPill, 1); header.Children.Add(portPill);

        var intro = new StackPanel { Spacing = 5 };
        intro.Children.Add(Text("Обновление прошивки", 28, true, 36)); intro.Children.Add(details);
        var fileBlock = new StackPanel { Spacing = 7 }; fileBlock.Children.Add(Text("Файл прошивки", 12));
        var fileGrid = new Grid { ColumnDefinitions = new("*,Auto"), Background = Field };
        var fileBorder = new Border { Child = fileGrid, BorderBrush = Line, BorderThickness = new(1), CornerRadius = new(8), Padding = new(14, 4) };
        firmwareName.VerticalAlignment = VerticalAlignment.Center; firmwareName.TextTrimming = TextTrimming.CharacterEllipsis; fileGrid.Children.Add(firmwareName);
        chooseFirmware.MinHeight = 40; chooseFirmware.Background = Brushes.Transparent; chooseFirmware.Foreground = Accent; chooseFirmware.FontWeight = FontWeight.SemiBold;
        Grid.SetColumn(chooseFirmware, 1); fileGrid.Children.Add(chooseFirmware); fileBlock.Children.Add(fileBorder);
        var backupCopy = new StackPanel { Spacing = 2 }; backupCopy.Children.Add(makeBackup); backupCopy.Children.Add(Text("Рекомендуется — позволит восстановить устройство", 12));
        var backupPanel = new Border { Child = backupCopy, Background = Brush.Parse("#0B2028"), BorderBrush = Brush.Parse("#216071"), BorderThickness = new(1), CornerRadius = new(8), Padding = new(14) };
        var progressHead = new Grid { ColumnDefinitions = new("*,Auto") }; progressHead.Children.Add(state); Grid.SetColumn(percent, 1); progressHead.Children.Add(percent);
        var progressBlock = new StackPanel { Spacing = 8 }; progressBlock.Children.Add(progressHead); progressBlock.Children.Add(progress);
        var note = Text("Во время обновления не отключайте USB.", 12); note.TextAlignment = TextAlignment.Center;
        var body = new StackPanel { Spacing = 20, Margin = new(44, 32) };
        foreach (var control in new Control[] { intro, fileBlock, backupPanel, progressBlock, update, note }) body.Children.Add(control);
        var root = new Grid { RowDefinitions = new("72,*") }; root.Children.Add(header); Grid.SetRow(body, 1); root.Children.Add(body); Content = root;

        chooseFirmware.Click += async (_, _) => await SelectFirmware(); update.Click += async (_, _) => await Run("install");
        portTimer.Tick += (_, _) => RefreshPort(); portTimer.Start(); RefreshPort();
        Closing += (_, e) => { if (busy && worker.Writing) { e.Cancel = true; details.Text = "Идёт запись или проверка. Дождитесь завершения."; } else StopWorker(); };
        Closed += (_, _) => portTimer.Stop();
        Opened += async (_, _) =>
        {
            if (Program.Arguments.Length == 2 && Program.Arguments[0] is "--preview" or "--smoke")
            {
                if (Program.Arguments[0] == "--smoke") await Run("check");
                await Task.Delay(500);
                using var bitmap = new RenderTargetBitmap(new PixelSize((int)Bounds.Width, (int)Bounds.Height), new Vector(96, 96));
                bitmap.Render(this); bitmap.Save(Program.Arguments[1], PngBitmapEncoderOptions.Default);
                await File.WriteAllTextAsync(Program.Arguments[1] + ".txt", state.Text + Environment.NewLine + details.Text); Close();
            }
        };
    }

    private static TextBlock Text(string value, double size, bool semibold = false, double? lineHeight = null) => new()
    {
        Text = value, FontSize = size, FontWeight = semibold ? FontWeight.SemiBold : FontWeight.Normal,
        Foreground = semibold ? Ink : Muted, LineHeight = lineHeight ?? (size <= 12 ? 16 : size <= 14 ? 20 : 36), TextWrapping = TextWrapping.Wrap
    };
    private static Button Button(string value) => new() { Content = value, MinHeight = 44, CornerRadius = new(8), Padding = new(16, 8) };

    private void RefreshPort()
    {
        if (busy) return;
        var matches = FindTillerPorts();
        selectedPort = matches.Count == 1 ? matches[0] : "";
        portStatus.Text = matches.Count switch { 0 => "Тиллер не найден", 1 => $"●  {selectedPort} найден", _ => "Найдено несколько тиллеров" };
        portStatus.Foreground = matches.Count == 1 ? Success : Muted;
        details.Text = matches.Count switch
        {
            0 => "Подключите тиллер по USB. Приложение найдёт его автоматически.",
            1 => "Тиллер подключён и готов к обновлению.",
            _ => "Оставьте подключённым только один тиллер."
        };
        state.Text = matches.Count == 1 ? "Готово к обновлению" : "Ожидание устройства";
        UpdateControls();
    }

    internal static List<string> FindTillerPorts()
    {
        var candidates = new List<(string Port, bool IsRunningTiller)>();
        using var activeKey = Registry.LocalMachine.OpenSubKey(@"HARDWARE\DEVICEMAP\SERIALCOMM");
        var active = activeKey?.GetValueNames().Select(name => activeKey.GetValue(name) as string)
            .Where(port => port != null).Cast<string>().ToHashSet(StringComparer.OrdinalIgnoreCase) ?? [];
        using var usb = Registry.LocalMachine.OpenSubKey(@"SYSTEM\CurrentControlSet\Enum\USB");
        if (usb == null) return [];
        foreach (var deviceName in usb.GetSubKeyNames().Where(x => x.StartsWith("VID_031F&PID_9001", StringComparison.OrdinalIgnoreCase) || x.StartsWith("VID_303A", StringComparison.OrdinalIgnoreCase)))
        using (var device = usb.OpenSubKey(deviceName))
        {
            if (device == null) continue;
            foreach (var instanceName in device.GetSubKeyNames())
            using (var parameters = device.OpenSubKey(instanceName + @"\Device Parameters"))
            {
                var port = parameters?.GetValue("PortName") as string;
                if (port != null && active.Contains(port) && Regex.IsMatch(port, "^COM[1-9][0-9]{0,3}$", RegexOptions.IgnoreCase))
                    candidates.Add((port.ToUpperInvariant(), deviceName.StartsWith("VID_031F&PID_9001", StringComparison.OrdinalIgnoreCase)));
            }
        }
        return DeviceDiscovery.PreferTillerPorts(candidates);
    }

    private async Task SelectFirmware()
    {
        var files = await StorageProvider.OpenFilePickerAsync(new() { Title = "Выберите файл прошивки", AllowMultiple = false, FileTypeFilter = [new("Прошивка ESP32") { Patterns = ["*.bin"] }] });
        var path = files.Count == 0 ? null : files[0].TryGetLocalPath(); if (path == null) return;
        firmwarePath = path; firmwareName.Text = Path.GetFileName(path); details.Text = "Файл выбран. Тиллер готов к обновлению."; UpdateControls();
    }

    private void UpdateControls()
    {
        update.IsEnabled = !busy && !fatal && selectedPort.Length != 0 && File.Exists(firmwarePath);
        chooseFirmware.IsEnabled = makeBackup.IsEnabled = !busy && !fatal;
    }

    private async Task Run(string action)
    {
        if (busy || fatal || (action != "check" && selectedPort.Length == 0)) return;
        worker = new(); busy = true; cancellation = new(); progress.Value = 0; percent.Text = "0%"; progress.IsIndeterminate = true;
        state.Text = action == "check" ? "Проверяю комплект" : "Подключение…"; details.Text = "Не отключайте USB до завершения операции."; UpdateControls();
        logPath = Path.Combine(DataRoot, "logs", DateTime.Now.ToString("yyyyMMdd-HHmmss-") + Guid.NewGuid().ToString("N")[..6] + ".log");
        try
        {
            var executable = Path.Combine(AppContext.BaseDirectory, "worker", "tiller-worker.exe");
            if (!File.Exists(executable)) throw new FileNotFoundException("Рабочий модуль не найден. Распакуйте весь архив.");
            var request = new { action, package = Path.Combine(AppContext.BaseDirectory, "package"), backups = Backups, port = selectedPort, baud = 460800, expected_mac = "", restore = "", make_backup = makeBackup.IsChecked == true, firmware_file = firmwarePath };
            await using var log = new StreamWriter(logPath, false, new UTF8Encoding(false));
            await Task.Run(() => worker.RunAsync(executable, request, log, e => Dispatcher.UIThread.Post(() => Event(e)), () => Task.FromResult(true), cancellation.Token));
        }
        catch (OperationCanceledException) { state.Text = "Операция остановлена"; details.Text = "Запись не начиналась."; }
        catch (Exception error) { state.Text = "Обновление не завершено"; details.Text = error.Message + (worker.Writing ? " Повторите обновление через BOOT + RST." : " Запись не начиналась."); }
        finally { busy = false; progress.IsIndeterminate = false; cancellation.Dispose(); cancellation = null; UpdateControls(); }
    }

    private void Event(JsonElement data)
    {
        var type = data.GetProperty("type").GetString(); if (data.TryGetProperty("message", out var message)) details.Text = message.GetString();
        switch (type)
        {
            case "device": state.Text = $"{data.GetProperty("port").GetString()} найден"; break;
            case "stage":
                var phase = data.GetProperty("phase").GetString(); state.Text = phase switch { "connect" => "Подключение…", "backup" => "Создание резервной копии", "verify" => "Проверка записи", _ => "Подготовка…" };
                progress.IsIndeterminate = phase is "connect" or "fingerprint" or "verify"; break;
            case "progress":
                progress.IsIndeterminate = false; var total = data.GetProperty("total").GetDouble(); var value = total > 0 ? Math.Clamp(data.GetProperty("current").GetDouble() * 100 / total, 0, 100) : 0;
                progress.Value = value; percent.Text = $"{value:0}%"; break;
            case "write_authorized": case "writing": state.Text = "Запись прошивки"; progress.IsIndeterminate = true; break;
            case "done": state.Text = "Готово"; progress.IsIndeterminate = false; progress.Value = 100; percent.Text = "100%"; break;
            case "error": progress.IsIndeterminate = false; break;
        }
    }

    public void StopWorker() => cancellation?.Cancel();
    public void ShowFatal(Exception error)
    {
        fatal = true; state.Text = "Ошибка приложения"; details.Text = "Операция остановлена. Перезапустите приложение.";
        try { File.AppendAllText(Path.Combine(DataRoot, "crash.log"), error + Environment.NewLine); } catch { } UpdateControls();
    }
}
