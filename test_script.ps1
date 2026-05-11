param(
    [string]$interface = "COM9",  # valeur par défaut
    [int]$channel = 15
)

# cargo embed --bin test_card --release --features defmt,tsch flash
probe-rs reset --chip nRF52840_xxAA

if ($LASTEXITCODE -ne 0) {
    Write-Host "[!] Flash échoué"
    exit 1
}

Write-Host "[+] Flash réussi, lancement du parsing..."

python validation_test_file.py $interface --channel $channel