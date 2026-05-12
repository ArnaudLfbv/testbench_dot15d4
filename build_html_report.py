from pathlib import Path
from datetime import datetime

def print_bilan(Tests_State, timestamp=None):
    col1 = max(len(k) for k in Tests_State.keys())
    col2 = len("Résultat")

    separator = f"+{'-' * (col1 + 2)}+{'-' * (col2 + 8)}+"
    header = f"| {'Nom':<{col1}} | {'Résultat':<{col2 + 6}} |"

    lines = [
        separator,
        header,
        separator
    ]

    # ---- HTML rows ----
    html_rows = []

    for name, result in Tests_State.items():
        status = "✅ PASS" if result else "❌ FAIL"

        lines.append(
            f"| {name:<{col1}} | {status:<{col2 + 6}} |"
        )

        color = "#d4edda" if result else "#f8d7da"

        html_rows.append(f"""
        <tr style="background-color:{color}">
            <td>{name}</td>
            <td>{status}</td>
        </tr>
        """)

    lines.append(separator)

    # ---- Terminal ----
    for line in lines:
        print(line)

    # ---- HTML report ----
    # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = Path(f"{timestamp}_bilan.html")

    html_content = f"""
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <title>Bilan des tests</title>

    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 40px;
            background-color: #f5f5f5;
        }}

        h1 {{
            color: #333;
        }}

        table {{
            border-collapse: collapse;
            width: 600px;
            background-color: white;
        }}

        th, td {{
            border: 1px solid #ccc;
            padding: 12px;
            text-align: left;
        }}

        th {{
            background-color: #333;
            color: white;
        }}
    </style>
</head>

<body>
    <h1>Bilan des tests</h1>

    <p><b>Date :</b> {timestamp}</p>

    <table>
        <tr>
            <th>Nom</th>
            <th>Résultat</th>
        </tr>

        {''.join(html_rows)}

    </table>
</body>
</html>
"""

    with open(filename, "w", encoding="utf-8") as f:
        f.write(html_content)

    print(f"\n[+] Rapport HTML sauvegardé : {filename}")