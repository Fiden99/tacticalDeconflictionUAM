from pathlib import Path

origine = Path("data/mercedesTD")
destinazione = Path("data/obtainedByHeur")

for file_origine in origine.rglob('*'):
    if file_origine.is_file():
        # Calcola il path relativo rispetto alla cartella origine
        relativo = file_origine.relative_to(origine)
        if "nDr1" in str(relativo):
            continue
        
        # Costruisce il path di destinazione mantenendo la struttura
        file_destinazione = destinazione / relativo

        # Crea le cartelle mancanti se necessario
        file_destinazione.parent.mkdir(parents=True, exist_ok=True)

        dir = file_origine.parent.name
        base_name = file_origine.stem
        if dir == "mercedesTD":
            pathFile = f"Path_{base_name}.txt"
        else:
            pathFile = f"{dir}/Path_{base_name}.txt"
        file_path = destinazione / pathFile
        contenuto = ""
        if file_path.exists():
            flights = set()
            with open(file_origine,"r") as file:
                riga = file.readline()
                while riga:
                    if "fixedFlights" in riga:
                        #leggo contentuto file originale e creo insieme di voli fissati
                        while ";" not in riga and riga:
                            riga = file.readline()
                            if ";" in riga:
                                break
                            flights.add(riga.strip().split(" ")[0]) 
                        #scrivo i voli fissati con path cambiato dall'euristica                         
                        contenuto +="set fixedFlights :=\n"
                        with open(file_path, "r") as file2:
                            riga2 = file2.readline()
                            while riga2:
                                flight = riga2.split(" ")[0]
                                if flight in flights:
                                    contenuto+= riga2.strip() + "\n"
                                riga2 = file2.readline()
                            contenuto +=";\n"
                    contenuto += riga.strip() + "\n"
                    riga = file.readline()
        else:
            print("file ",file_path," with updated path from the heuristic not found")   
            continue    
        file_destinazione.write_text(contenuto)
