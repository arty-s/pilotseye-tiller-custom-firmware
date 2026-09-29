using System.Security.Cryptography;
using System.Text.Json;

namespace Tiller;

public sealed record BackupEntry(string Folder, string Mac, string Saved, bool Complete, int Blocks)
{
    public override string ToString() => $"{(Complete ? "✓ Проверен" : $"↻ Докачка · {Blocks}/64 блоков")}   {Saved}   {Mac}";
}

public static class BackupStore
{
    public static List<BackupEntry> List(string root)
    {
        Directory.CreateDirectory(root);
        var result = new List<BackupEntry>();
        foreach (var folder in Directory.EnumerateDirectories(root).OrderDescending())
        {
            try
            {
                var complete = File.Exists(Path.Combine(folder, "backup.json"));
                using var doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(folder, complete ? "backup.json" : "resume.json")));
                var meta = doc.RootElement;
                result.Add(new(folder, meta.GetProperty("mac").GetString() ?? "?",
                    complete ? meta.GetProperty("saved").GetString() ?? "" : Directory.GetCreationTime(folder).ToString("dd.MM.yyyy HH:mm"),
                    complete, complete ? 64 : meta.GetProperty("chunks").EnumerateObject().Count()));
            }
            catch (Exception e) when (e is IOException or JsonException or KeyNotFoundException or InvalidOperationException) { }
        }
        return result;
    }
    public static async Task Import(string metaPath, string root)
    {
        using var doc = JsonDocument.Parse(await File.ReadAllTextAsync(metaPath));
        var meta = doc.RootElement;
        var source = Path.Combine(Path.GetDirectoryName(Path.GetFullPath(metaPath))!, "before-flash.bin");
        if (meta.GetProperty("bytes").GetInt64() != 16777216 || new FileInfo(source).Length != 16777216)
            throw new InvalidDataException("Бэкап должен содержать ровно 16 МиБ.");
        var mac = meta.GetProperty("mac").GetString();
        if (mac == null || !System.Text.RegularExpressions.Regex.IsMatch(mac, "^[a-fA-F0-9]{2}(:[a-fA-F0-9]{2}){5}$"))
            throw new InvalidDataException("В бэкапе нет корректного MAC.");
        var bytes = await File.ReadAllBytesAsync(source);
        if (!Convert.ToHexString(SHA256.HashData(bytes)).Equals(meta.GetProperty("sha256").GetString(), StringComparison.OrdinalIgnoreCase))
            throw new InvalidDataException("Контрольная сумма бэкапа не совпадает.");
        var folder = Path.Combine(root, "import-" + DateTime.Now.ToString("yyyyMMdd-HHmmss-") + Guid.NewGuid().ToString("N")[..8]);
        Directory.CreateDirectory(folder);
        await File.WriteAllBytesAsync(Path.Combine(folder, "before-flash.bin"), bytes);
        // Commit metadata last; interrupted imports never appear as usable restore images.
        await File.WriteAllTextAsync(Path.Combine(folder, "backup.json"), doc.RootElement.GetRawText());
    }
}
