import cv2,math,time

class Tack_Object:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0
        self.counter_obj = ""
        self.cnt_object = 0
    
    def update(self, objects_rect):
        # Objects boxes and ids
        objects_bbs_ids = []

        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            # Find out if that object was detected already
            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 25:
                    self.center_points[id] = (cx, cy)
                    #print(self.center_points)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True
                    break

            # New object is detected we assign the ID to that object
            if same_object_detected is False:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1
            
        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()
        return objects_bbs_ids

    def track_object(self, Image, HSVLower, HSVUpper, Color):
        
        self.chk_detect = False
        self.curr_object = 0
        org_hsv = cv2.cvtColor(src=Image,code=cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(src=org_hsv,lowerb=HSVLower,upperb=HSVUpper)

        detection = []
        
        (conts,hierarchy) = cv2.findContours(image=color_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        for obj_cnts in conts:
            (x,y,w,h) = cv2.boundingRect(obj_cnts)
            if cv2.contourArea(obj_cnts) > 500:
                self.chk_detect = True
                self.curr_object += 1
                Image = cv2.rectangle(img=Image,pt1=(x,y),pt2=(x+w,y+h),color=Color,thickness=2)
                detection.append([x, y, w, h])
        
        boxes_ids = self.update(detection)

        for box_id in boxes_ids:
            x, y, w, h, id = box_id
            Image = cv2.putText(img=Image,
                                text=str(f"ID:{id+1}"),
                                org=(x, y - 15),
                                fontFace=cv2.FONT_HERSHEY_PLAIN,
                                fontScale=1,
                                color=(0, 255, 0),
                                thickness=2)
            self.cnt_object = id+1
        return Image

    def get_total(self):
        return self.cnt_object
    def get_current(self):
        return self.curr_object
    def get_IsDetect(self):
        return self.chk_detect
