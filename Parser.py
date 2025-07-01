
import csv

#style beispiel für die einzulesende Datei
#
#numberOfAgents;gridSize;logic;viewDistanz;viewAngle;stepsPerTime;seed;lambdaDistance;lambdaLength
#1;12;3;4;5;6;7
#1;12;3;4;5;6;7
#1;12;3;4;5;6;7
#

#Encoding fuer CSV datei
encoding="utf-8"

#Spaltenüberschriften *Muss mit Zeilenüberschriften in der CSV-Datei uebereinstimmen*
value1 = "numberOfAgents"
value2 = "gridSize"
value3 = "logic"
value4 = "viewDistanz"
value5 = "viewAngle"
value6 = "stepsPerTime"
value7 = "seed"
value8 = "lambdaDistance"
value9 = "lambdaLength"


#input: Pfad zur CSV Datei
#Output: Dictionary mit Listen der einzellen eingelesenen Werte

def load_parameters_from_csv(file_path):
    parameters = {value1: [],
                  value2: [],
                  value3: [],
                  value4: [],
                  value5: [],
                  value6: [],
                  value7: [],
                  value8: [],
                  value9: []}
    with open(file_path, newline='', encoding=encoding) as csvfile:
        reader = csv.DictReader(csvfile, delimiter=';')
        for row in reader:
            parameters[value1].append(int(row[value1]))
            parameters[value2].append(int(row[value2]))
            parameters[value3].append(int(row[value3]))
            parameters[value4].append(int(row[value4]))
            parameters[value5].append(int(row[value5]))
            parameters[value6].append(int(row[value6]))
            parameters[value7].append(int(row[value7]))
            parameters[value7].append(int(row[value8]))
            parameters[value7].append(int(row[value9]))
    return parameters

