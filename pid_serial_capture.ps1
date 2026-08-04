$port = [System.IO.Ports.SerialPort]::new('COM3', 115200, 'None', 8, 'One')
$port.ReadTimeout = 200
$logPath = Join-Path $PSScriptRoot 'pid_com5_latest.log'
$writer = [System.IO.StreamWriter]::new($logPath, $false, [System.Text.Encoding]::UTF8)

try {
    $port.Open()
    $deadline = (Get-Date).AddMinutes(5)
    while ((Get-Date) -lt $deadline) {
        $data = $port.ReadExisting()
        if ($data.Length -gt 0) {
            $writer.Write($data)
            $writer.Flush()
        }
        Start-Sleep -Milliseconds 20
    }
}
finally {
    $writer.Dispose()
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
