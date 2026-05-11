#!/usr/bin/env bash

INTERFACE="${1:-COM9}"
CHANNEL="${2:-15}"

# Se placer dans le dossier du script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

echo "[+] Lancement du parsing en arrière-plan..."

python3 validation_test_file.py "$INTERFACE" --channel "$CHANNEL" &
PYTHON_PID=$!

sleep 5

echo "[+] Reset de la carte..."

# cargo embed --bin test_card --release --features defmt,tsch flash
probe-rs reset --chip nRF52840_xxAA

if [ $? -ne 0 ]; then
    echo "[!] Reset échoué"
    kill $PYTHON_PID 2>/dev/null
    exit 1
fi

echo "[+] Reset réussi"

# Attendre que le script Python termine
wait $PYTHON_PID

read -p "Appuie sur Entrée pour quitter..."