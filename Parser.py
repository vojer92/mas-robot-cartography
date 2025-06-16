
import csv

#style beispiel für die einzulesende Datei
#
#anzahl,gridGroesse,sichtDistanz,sichtWinkel,Platzhalter1,Platzhalter2,Platzhalter3
#1,2,3,4,5,6,7
#8,9,10,11,12,13,14
#15,16,17,18,19,20,21"
#

#Encoding fuer CSV datei
encoding="utf-8"

#Spaltenüberschriften *Muss mit Zeilenüberschriften in der CSV-Datei uebereinstimmen*
value1 = "anzahl"
value2 = "gridGroesse"
value3 = "sichtDistanz"
value4 = "sichtWinkel"
value5 = "Platzhalter1"
value6 = "Platzhalter2"
value7 = "Platzhalter3"

#input: Pfad zur CSV Datei
#Output: Dictionary mit Listen der einzellen eingelesenen Werte

def load_parameters_from_csv(file_path):
    parameters = {value1: [],
                  value2: [],
                  value3: [],
                  value4: [],
                  value5: [],
                  value6: [],
                  value7: []}
    with open(file_path, newline='', encoding=encoding) as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            parameters[value1].append(int(row[value1]))
            parameters[value2].append(int(row[value2]))
            parameters[value3].append(int(row[value3]))
            parameters[value4].append(int(row[value4]))
            parameters[value5].append(int(row[value5]))
            parameters[value6].append(int(row[value6]))
            parameters[value7].append(int(row[value7]))
    return parameters

