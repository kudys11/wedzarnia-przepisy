# To jest przykładowy przepis na wędzenie łososia
# Linie zaczynające się od # są ignorowane przez program.

# Krok 1: Suszenie
Suszenie;30.0;0;120;1;0;1;0;0;0

# Krok 2: Wędzenie (faza 1)
Wedzenie_1;60.0;0;180;2;120;2;10;60;0

# Krok 3: Wędzenie (faza 2) - wyższa temperatura
Wedzenie_2;70.0;0;120;3;180;1;0;0;0

# Krok 4: Pieczenie/parzenie (dla temperatury mięsa)
Parzenie;80.0;65.0;60;3;0;1;0;0;1

# Krok 5: Studzenie (nie grzejemy, tylko wentylujemy)
Studzenie;20.0;0;30;0;0;1;0;0;0
Jak interpretować ten przykładowy przepis:
Suszenie:

Nazwa: Suszenie

T. komory: 30°C

T. mięsa: 0 (nie używana)

Min. czas: 120 minut (2 godziny)

Moc: 1 grzałka

Dym: 0 (wyłączony)

Wentylator: ON (ciągle)

Fan ON/OFF: 0 (nie dotyczy)

Użyj T. mięsa: 0 (nie)

Wędzenie (faza 1):

Nazwa: Wedzenie_1

T. komory: 60°C

T. mięsa: 0 (nie używana)

Min. czas: 180 minut (3 godziny)

Moc: 2 grzałki

Dym: 120 (średnia moc)

Wentylator: CYKLiCZNY

Fan ON: 10 sekund

Fan OFF: 60 sekund

Użyj T. mięsa: 0 (nie)

Wędzenie (faza 2) - wyższa temperatura:

Nazwa: Wedzenie_2

T. komory: 70°C

T. mięsa: 0 (nie używana)

Min. czas: 120 minut (2 godziny)

Moc: 3 grzałki

Dym: 180 (wysoka moc)

Wentylator: ON (ciągle)

Fan ON/OFF: 0 (nie dotyczy)

Użyj T. mięsa: 0 (nie)

Pieczenie/parzenie (dla temperatury mięsa):

Nazwa: Parzenie

T. komory: 80°C

T. mięsa: 65.0°C (cel)

Min. czas: 60 minut (1 godzina)

Moc: 3 grzałki

Dym: 0 (wyłączony)

Wentylator: ON (ciągle)

Fan ON/OFF: 0 (nie dotyczy)

Użyj T. mięsa: 1 (tak) - to kluczowe! Krok zakończy się dopiero, gdy minie 60 minut i temperatura mięsa osiągnie 65.0°C.

Studzenie:

Nazwa: Studzenie

T. komory: 20°C (schładzamy do tej temp. lub po prostu nic nie grzejemy)

T. mięsa: 0 (nie używana)

Min. czas: 30 minut

Moc: 0 (wyłączone grzałki)

Dym: 0 (wyłączony)

Wentylator: ON (ciągle - dla chłodzenia)

Fan ON/OFF: 0 (nie dotyczy)

Użyj T. mięsa: 0 (nie)
