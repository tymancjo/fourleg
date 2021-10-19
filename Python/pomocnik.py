

lista_komend = """
<42,109,83,81,97,109,73,71,97>;
<42,109,83,81,97,109,73,36,136>;
<42,109,83,81,97,109,73,66,71>;
<42,93,59,97,121,93,49,66,71>;
<42,138,71,97,121,93,49,66,71>;
<42,101,108,97,121,93,49,66,71>;
<42,82,103,97,121,93,49,66,71>;
<42,82,103,97,121,135,56,66,71>;
<42,82,103,97,121,111,104,66,71>;
<42,83,59,107,121,83,49,97,121>;
<42,83,59,59,149,83,49,97,121>;
<42,83,59,107,121,83,49,97,121>
"""

lista = lista_komend.strip().split(";")
kroki = []

for line in lista:
    servos = line[5:-1].strip().split(",")
    kroki.append(servos)

out = "{\n\r"
for line in kroki:
    msg = "{"
    for servo in line:
        msg += f"{servo.strip()},"
    msg = msg[:-1] + "},\n\r"
    out += msg
out = out[:-1] + "}"


print(out)
print(f"len: {len(lista)}")
