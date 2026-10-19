#Requires -RunAsAdministrator
[CmdletBinding(SupportsShouldProcess = $true)]
param(
    [ValidateSet('Status', 'Enable', 'Disable')]
    [string]$Action = 'Status'
)

$ErrorActionPreference = 'Stop'
$hostRule = 'AS2-VLM-Robot-UDP-Ethernet'
$vmRule = 'AS2-VLM-Robot-UDP-WSL'
$peer = '192.168.123.161'
$wslId = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'

if ($Action -eq 'Status') {
    Get-NetFirewallRule -Name $hostRule -ErrorAction SilentlyContinue |
        Select-Object Name, Enabled, Direction, Action
    Get-NetFirewallHyperVRule -Name $vmRule -ErrorAction SilentlyContinue |
        Select-Object Name, Enabled, Direction, Action
    return
}

if ($Action -eq 'Disable') {
    if ($PSCmdlet.ShouldProcess('Only AS2-VLM named rules', 'Remove peer UDP permissions')) {
        Get-NetFirewallHyperVRule -Name $vmRule -ErrorAction SilentlyContinue |
            Remove-NetFirewallHyperVRule
        Get-NetFirewallRule -Name $hostRule -ErrorAction SilentlyContinue |
            Remove-NetFirewallRule
    }
    return
}

$adapter = Get-NetAdapter -Name Ethernet
if ($adapter.InterfaceDescription -ne 'Realtek Gaming GbE Family Controller') {
    throw 'Ethernet adapter differs from the verified setup. Review the interface before enabling rules.'
}
if ((Get-NetFirewallRule -Name $hostRule -ErrorAction SilentlyContinue) -or
    (Get-NetFirewallHyperVRule -Name $vmRule -ErrorAction SilentlyContinue)) {
    throw 'Named rules already exist. Inspect Status or remove them with Disable before reconfiguration.'
}

if ($PSCmdlet.ShouldProcess("Inbound UDP from $peer on Ethernet and in WSL", 'Create persistent firewall permissions')) {
    $hostCreated = $false
    try {
        # DDS uses negotiated UDP ports; a fixed 7400-7500 range was insufficient.
        New-NetFirewallRule -Name $hostRule -DisplayName 'AS2 VLM robot UDP on Ethernet' `
            -InterfaceAlias Ethernet -Direction Inbound -Action Allow -Protocol UDP `
            -RemoteAddress $peer -Profile Any | Out-Null
        $hostCreated = $true
        New-NetFirewallHyperVRule -Name $vmRule -DisplayName 'AS2 VLM robot UDP in WSL' `
            -VMCreatorId $wslId -Direction Inbound -Action Allow -Protocol UDP `
            -RemoteAddresses $peer | Out-Null
    } catch {
        if ($hostCreated) { Remove-NetFirewallRule -Name $hostRule }
        throw
    }
    Write-Output "Enabled inbound UDP only from $peer. Global firewall policies are unchanged."
    Write-Output 'Disable these rules after disconnecting from the trusted robot network.'
}
