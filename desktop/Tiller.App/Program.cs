using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Themes.Fluent;
using Avalonia.Styling;
using Avalonia.Threading;

namespace Tiller;

internal static class Program
{
    public static string[] Arguments = [];
    [STAThread]
    public static void Main(string[] args)
    {
        Arguments = args;
        using var instance = new Mutex(true, @"Local\TillerDesktop-SerialOwner", out var first);
        if (!first) return;
        try { BuildAvaloniaApp().StartWithClassicDesktopLifetime(args); }
        catch (Exception error)
        {
            Directory.CreateDirectory(MainWindow.DataRoot);
            File.AppendAllText(Path.Combine(MainWindow.DataRoot, "crash.log"), error + Environment.NewLine);
            throw;
        }
    }
    public static AppBuilder BuildAvaloniaApp() => AppBuilder.Configure<App>().UsePlatformDetect();
}

public sealed class App : Application
{
    public override void Initialize()
    {
        RequestedThemeVariant = ThemeVariant.Dark;
        Styles.Add(new FluentTheme());
        Styles.Add(new Style(x => x.OfType<Avalonia.Controls.Button>())
        {
            Setters = { new Setter(Avalonia.Controls.Button.MinHeightProperty, 44d),
                new Setter(Avalonia.Controls.Button.PaddingProperty, new Thickness(16, 8)),
                new Setter(Avalonia.Controls.Button.CornerRadiusProperty, new CornerRadius(6)) }
        });
    }
    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            var window = new MainWindow();
            desktop.MainWindow = window;
            desktop.Exit += (_, _) => window.StopWorker();
            Dispatcher.UIThread.UnhandledException += (_, e) =>
            {
                window.StopWorker();
                window.ShowFatal(e.Exception);
                e.Handled = true;
            };
        }
        base.OnFrameworkInitializationCompleted();
    }
}
