namespace Tiller;

public static class DeviceDiscovery
{
    public static List<string> PreferTillerPorts(IEnumerable<(string Port, bool IsRunningTiller)> candidates)
    {
        var devices = candidates.ToList();
        var running = devices.Where(candidate => candidate.IsRunningTiller).Select(candidate => candidate.Port)
            .Distinct(StringComparer.OrdinalIgnoreCase).Order().ToList();
        return running.Count != 0
            ? running
            : devices.Select(candidate => candidate.Port).Distinct(StringComparer.OrdinalIgnoreCase).Order().ToList();
    }
}
