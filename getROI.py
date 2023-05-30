import json

def returnROI():
    with open('data.json', 'r') as json_file:
        roi_data = json.load(json_file)
    
    rois = roi_data["ROI"]

    print("The Region of interest with their id are:")
    for roi in rois:
        roi_id = roi["ID"]
        print(roi_id)



if __name__ == "__main__":
    returnROI()
