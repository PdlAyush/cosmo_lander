import json


def returnPOI():

    with open('data.json', 'r') as json_file:
        roi_data = json.load(json_file)
        
    rois = roi_data["ROI"]

    for roi in rois:
        print("For roi with id '{}', the point of interests with their PID are: ".format(roi['ID']))
        poi_list = roi['Point of Interest']
        for poi in poi_list:
            pid = poi['PID']
            print(pid)
        print("\n")


if __name__ == "__main__":
    returnPOI()