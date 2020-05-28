import cv2
import numpy as np

from mavsdk_utils import get_location_offset_meters

np.random.seed(2)
COLORS = np.random.randint(0, 255, size=(91, 3), dtype="uint8")

import plotly.graph_objects as go
mapbox_access_token = "pk.eyJ1IjoiYWhtZWRoaXNoYW0iLCJhIjoiY2s5MTY5emt3MDgzMTNrbzg1cW1mZDVzeSJ9.PshMTyiTONnbOxELBBi-FQ"

def draw_outputs(img, outputs, class_names, detector='ssd'):
    if detector == 'ssd':
        boxes, objectness, classes, nums = outputs
        h, w = img.shape[0:2]
        for i in range(nums):
            top, left, bottom, right = boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]
            x1y1 = (int(w*left), int(h*top))
            x2y2 = (int(w*right), int(h*bottom))
            color = [int(c) for c in COLORS[classes[i]]]
            img = cv2.rectangle(img, x1y1, x2y2, color, 2)
            img = cv2.putText(img, '{} {:.2f}'.format(
                class_names[classes[i]]['name'], objectness[i]),
                x1y1, cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.6, (255, 255, 255), 1)
    else:
        boxes, objectness, classes, nums = outputs
        wh = np.flip(img.shape[0:2])
        for i in range(nums):
            x1y1 = tuple((np.array(boxes[i][0:2]) * wh).astype(np.int32))
            x2y2 = tuple((np.array(boxes[i][2:4]) * wh).astype(np.int32))
            img = cv2.rectangle(img, x1y1, x2y2, (255, 0, 0), 2)
            img = cv2.putText(img, '{} {:.4f}'.format(
                class_names[int(classes[i])], objectness[i]),
                x1y1, cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)
    return img

def keep_person_only(boxes_squeezed, scores_squeezed, classes_squeezed, confidence=0.5, detector='ssd'):
    if detector == 'ssd':
        idx = np.logical_and(classes_squeezed == 1, scores_squeezed >= confidence)
    else:
        idx = np.logical_and(classes_squeezed == 0, scores_squeezed >= confidence)
    nums = len(classes_squeezed[idx])     # Number of detected persons with confidence
    return boxes_squeezed[idx], scores_squeezed[idx], classes_squeezed[idx], nums


def show_in_window(fig):
    ''' this is a helper function to coordinates_plot '''
    import sys, os
    import plotly.offline
    from PyQt5.QtCore import QUrl
    from PyQt5.QtWebEngineWidgets import QWebEngineView
    from PyQt5.QtWidgets import QApplication

    plotly.offline.plot(fig, filename='map.html', auto_open=False)

    app = QApplication(sys.argv)
    web = QWebEngineView()
    file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "map.html"))
    web.load(QUrl.fromLocalFile(file_path))
    web.show()
    sys.exit(app.exec_())

def coordinates_plot(person_lat, person_lon, home_lat, home_lon):
    lats = [person_lat, home_lat]
    lons = [person_lon, home_lon]
    fig = go.Figure(go.Scattermapbox(
        lat=lats,
        lon=lons,
        mode='markers',
        marker={'size': 15, 'symbol': ['clothing-store', 'airport']},
        text=['Average Person Location','UAV Home Location'],))
    fig.update_layout(
        hovermode='closest',
        mapbox=dict(
            accesstoken=mapbox_access_token,
            style='mapbox://styles/ahmedhisham/ck916b6j400701ipm66znjg2c',
            bearing=0,
            center=go.layout.mapbox.Center(lat=lats[-1],lon=lons[-1]),
            pitch=0,
            zoom=16))
    show_in_window(fig)

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

def localize_person(img, boxes, nums, alt, yaw, drone_lat, drone_lon):
    hfov = 2            # rad
    vfov = 82*3.14/180  # rad = 1.43
    yaw = yaw - 90
    gnd_width_m = alt*2*np.tan(hfov/2)
    gnd_height_m = alt*2*np.tan(vfov/2)
    h, w = rotate_image(img, -yaw).shape[0:2]
    m_to_pixels_w = gnd_width_m/w
    m_to_pixels_h = gnd_height_m/h
    # lists to return
    person_lats = []
    person_lons = []
    for box in boxes:
        # Note: North = y = height = h      &       East = x = width = w
        center_x, center_y = int(w*(box[3]+box[1])/2), int(h*(box[2]+box[0])/2)
        dY, dX = h//2 - center_y, center_x - w//2   # difference in pixels; Y(+ve if above center), X(+ve if east of center)
        print(f'yaw:{yaw}')
        print(f'dX:{dX}, dY:{dY}')
        dN_m, dE_m = m_to_pixels_h*dY, m_to_pixels_w*dX
        print(f'dN_m:{dN_m}, dE_m:{dE_m}')
        person_lat, person_lon = get_location_offset_meters(original_lat=drone_lat, original_lon=drone_lon, 
                                                            dNorth=dN_m, dEast=dE_m)
        print(f'A person is detected at:\n{person_lat}, {person_lon}')
        img = cv2.putText(img, f'lat:{person_lat}, lon:{person_lon}', (center_x,center_y), 
                          cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.3, 
                          (255,255,255), 1)
        person_lats.append(person_lat)
        person_lons.append(person_lon)
    cv2.imshow('rotated image', rotate_image(img, -yaw))
    return person_lats, person_lons
