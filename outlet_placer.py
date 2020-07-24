import json

def place_outlets(studio, floors):
    # write me!
    outlets = []
    return outlets

def main():
    # room info
    studio = "json/studio_info.json"
    with open(studio) as json_file:
        studio = json.load(json_file)

    # floor info
    floors = "json/floors_info.json"
    with open(floors) as json_file:
        floors = json.load(json_file)

    outlets = place_outlets(studio, floors)
    with open('outlets.json', 'w') as json_out:
        json.dump(outlets, json_out)

if __name__ == "__main__":
    main()