from hashlib import new
from PyQt5 import uic
from PyQt5.QtMultimedia import QCameraInfo
from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QTimer, QDateTime, Qt
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen
import cv2,time,sys,sysinfo
import numpy as np
import random as rnd
from Tracking_Func import Tack_Object
#import RPi.GPIO as IO


class ThreadClass(QThread):
    ImageUpdate = pyqtSignal(np.ndarray)
    FPS = pyqtSignal(int)
    global camIndex

    def run(self):
        if camIndex == 0:
            Capture = cv2.VideoCapture(camIndex)
        if camIndex == 1:
            Capture = cv2.VideoCapture(camIndex,cv2.CAP_DSHOW)

        Capture.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        Capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.ThreadActive = True
        prev_frame_time = 0
        new_frame_time = 0
        while self.ThreadActive: 
            ret,frame_cap = Capture.read()
            flip_frame = cv2.flip(src=frame_cap,flipCode=-1)
            new_frame_time = time.time()
            fps = 1/(new_frame_time-prev_frame_time)
            prev_frame_time = new_frame_time
            if ret:
                self.ImageUpdate.emit(flip_frame)
                self.FPS.emit(fps)
    
    def stop(self):
        self.ThreadActive = False
        self.quit()
           
class boardInfoClass(QThread):
    cpu = pyqtSignal(float)
    ram = pyqtSignal(tuple)
    temp = pyqtSignal(float)
    
    def run(self):
        self.ThreadActive = True
        while self.ThreadActive:
            cpu = sysinfo.getCPU()
            ram = sysinfo.getRAM()
            #temp = sysinfo.getTemp()
            self.cpu.emit(cpu)
            self.ram.emit(ram)
            #self.temp.emit(temp)

    def stop(self):
        self.ThreadActive = False
        self.quit()

class randomColorClass(QThread):
    color = pyqtSignal(tuple)
    def run(self):
        self.ThreadActive = True
        while self.ThreadActive:
            color = ([rnd.randint(0,256),rnd.randint(0,256),rnd.randint(0,256)],
                     [rnd.randint(0,256),rnd.randint(0,256),rnd.randint(0,256)],
                     [rnd.randint(0,256),rnd.randint(0,256),rnd.randint(0,256)]
                     )
            self.color.emit(color)
            time.sleep(2)

    def stop(self):
        self.ThreadActive = False
        self.quit()
        
class Window_IOMonitor(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("IO_monitor.ui",self)

class Window_ErrorAlarm(QDialog):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("Error.ui", self)

#   QLabel display
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("Opencv_PiDash.ui",self)
        
        self.online_cam = QCameraInfo.availableCameras()
        self.camlist.addItems([c.description() for c in self.online_cam])
        self.btn_start.clicked.connect(self.StartWebCam)
        self.btn_stop.clicked.connect(self.StopWebcam)

        self.resource_usage = boardInfoClass()
        self.resource_usage.start()
        self.resource_usage.cpu.connect(self.getCPU_usage)
        self.resource_usage.ram.connect(self.getRAM_usage)
        #self.resource_usage.temp.connect(self.getTemp_usage)

        self.randomColor_usage = randomColorClass()
        self.randomColor_usage.start()
        self.randomColor_usage.color.connect(self.get_randomColors)

# Create Instance class
        self.Win_showIO = Window_IOMonitor()
        self.Win_showError = Window_ErrorAlarm()

        # Track object Functions
        self.Track_Function1 = Tack_Object()
        self.Track_Function2 = Tack_Object()
        self.Track_Function3 = Tack_Object()



# QTimer Zone
        self.ready_lamp = QTimer(self, interval=1000)
        self.ready_lamp.timeout.connect(self.Ready_lamp)
        self.ready_lamp.start()

        self.motor_on = QTimer(self, interval=1000)
        self.motor_on.timeout.connect(self.Running_lamp)
        self.motor_on.timeout.connect(self.Motor_state)

        self.lcd_timer = QTimer()
        self.lcd_timer.timeout.connect(self.clock)
        self.lcd_timer.start() 

        self.flag_motor = True
        self.Status_lamp = [True,True,True]
# End QTimer Zone  

        self.Motor_status.setPixmap(QPixmap('Icons/MotorOFF.png'))
        self.btn_motor.setText('Motor ON')

        
        self.btn_motor.setCheckable(True)
        self.btn_motor.clicked.connect(self.Motor_on)
        
        self.btn_setObject1.setCheckable(True)
        self.btn_setObject1.clicked.connect(self.GetObject_one)

        self.btn_setObject2.setCheckable(True)
        self.btn_setObject2.clicked.connect(self.GetObject_two)

        self.btn_setObject3.setCheckable(True)
        self.btn_setObject3.clicked.connect(self.GetObject_three)
        
        self.btn_close.clicked.connect(self.Close_software)
        self.btn_iomonitor.clicked.connect(self.open_IOmonitor)

        self.btn_roi_set.setCheckable(True)
        self.btn_roi_set.clicked.connect(self.set_roi)
        
        self.ROI_X.valueChanged.connect(self.get_ROIX)
        self.ROI_Y.valueChanged.connect(self.get_ROIY)
        self.ROI_W.valueChanged.connect(self.get_ROIW)
        self.ROI_H.valueChanged.connect(self.get_ROIH)
        self.ckb_roi.setChecked(True)

        self.roi_x = 20
        self.roi_y = 20
        self.roi_w = 2000
        self.roi_h = 2000
 
        self.Win_showError.btn_e_close.clicked.connect(self.Close_Error)
        self.btn_stop.setEnabled(False)

    def get_randomColors(self,color):
        self.RanColor1 = color[0]
        self.RanColor2 = color[1]
        self.RanColor3 = color[2]

    def getCPU_usage(self,cpu):
        self.Qlabel_cpu.setText(str(cpu) + " %")
        if cpu > 15: self.Qlabel_cpu.setStyleSheet("color: rgb(23, 63, 95);")
        if cpu > 25: self.Qlabel_cpu.setStyleSheet("color: rgb(32, 99, 155);")
        if cpu > 45: self.Qlabel_cpu.setStyleSheet("color: rgb(60, 174, 163);")
        if cpu > 65: self.Qlabel_cpu.setStyleSheet("color: rgb(246, 213, 92);")
        if cpu > 85: self.Qlabel_cpu.setStyleSheet("color: rgb(237, 85, 59);")

    def getRAM_usage(self,ram):
        self.Qlabel_ram.setText(str(ram[2]) + " %")
        if ram[2] > 15: self.Qlabel_ram.setStyleSheet("color: rgb(23, 63, 95);")
        if ram[2] > 25: self.Qlabel_ram.setStyleSheet("color: rgb(32, 99, 155);")
        if ram[2] > 45: self.Qlabel_ram.setStyleSheet("color: rgb(60, 174, 163);")
        if ram[2] > 65: self.Qlabel_ram.setStyleSheet("color: rgb(246, 213, 92);")
        if ram[2] > 85: self.Qlabel_ram.setStyleSheet("color: rgb(237, 85, 59);")

    def getTemp_usage(self,temp):
        self.Qlabel_temp.setText(str(temp) + " *C")
        if temp > 30: self.Qlabel_temp.setStyleSheet("color: rgb(23, 63, 95);")
        if temp > 35: self.Qlabel_temp.setStyleSheet("color: rgb(60, 174, 155);")
        if temp > 40: self.Qlabel_temp.setStyleSheet("color: rgb(246,213, 92);")
        if temp > 45: self.Qlabel_temp.setStyleSheet("color: rgb(237, 85, 59);")
        if temp > 50: self.Qlabel_temp.setStyleSheet("color: rgb(255, 0, 0);")
        
    def get_FPS(self,fps):
        self.Qlabel_fps.setText(str(fps))
        if fps > 5: self.Qlabel_fps.setStyleSheet("color: rgb(237, 85, 59);")
        if fps > 15: self.Qlabel_fps.setStyleSheet("color: rgb(60, 174, 155);")
        if fps > 25: self.Qlabel_fps.setStyleSheet("color: rgb(85, 170, 255);")
        if fps > 35: self.Qlabel_fps.setStyleSheet("color: rgb(23, 63, 95);")

    def clock(self):
        self.DateTime = QDateTime.currentDateTime()
        self.lcd_clock.display(self.DateTime.toString('hh:mm:ss'))

# Open I/O Window monitor 
    def open_IOmonitor(self):
        self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Open I/O Monitor")
        self.Win_showIO.show()

# Close Error Notification window
    def Close_Error(self):
        self.Win_error.close()

# ! Function oneclick to hsv parameter
    def GetObject_one(self):
        try:
            self.hsvOne_lower = np.array([self.Slider_H_min.value(), self.Slider_S_min.value(), self.Slider_V_min.value()], dtype=np.uint8)
            self.hsvOne_upper = np.array([self.Slider_H_max.value(), self.Slider_S_max.value(), self.Slider_V_max.value()], dtype=np.uint8)
            self.textEdit.append(f"Object 1 >> HSV Lower: {self.hsvOne_lower} | HSV Upper: {self.hsvOne_upper}")
            # For Debug
            #print(f"HSV Lower: {self.hsvOne_lower} | HSV Upper: {self.hsvOne_upper}")
            hsv2mask_one = cv2.cvtColor(src=self.CopyImage,code=cv2.COLOR_BGR2HSV)
            mask_object1_range = cv2.inRange(src=hsv2mask_one,lowerb=self.hsvOne_lower,upperb=self.hsvOne_upper)
            mask_object1 = cv2.bitwise_and(src1=self.CopyImage,src2=self.CopyImage,mask=mask_object1_range)
            mask_disp1 = self.cvt_cv_qt(mask_object1)
            self.disp_obj1.setPixmap(mask_disp1)
            self.disp_obj1.setScaledContents(True)
        except:
            self.Win_error.show()
            self.Win_error.Qlabel_error.setText("Please start camera before set !")

# ! Function oneclick to hsv parameter        
    def GetObject_two(self):
        try:
            self.hsvTwo_lower = np.array([self.Slider_H_min.value(), self.Slider_S_min.value(), self.Slider_V_min.value()], dtype=np.uint8)
            self.hsvTwo_upper = np.array([self.Slider_H_max.value(), self.Slider_S_max.value(), self.Slider_V_max.value()], dtype=np.uint8)
            self.textEdit.append(f"Object 2 >> HSV Lower: {self.hsvTwo_lower} | HSV Upper: {self.hsvTwo_upper}")
            # For Debug
            #print(f"HSV Lower: {self.hsvOne_lower} | HSV Upper: {self.hsvOne_upper}")

            hsv2mask_two = cv2.cvtColor(src=self.CopyImage,code=cv2.COLOR_BGR2HSV)
            mask_object2_range = cv2.inRange(src=hsv2mask_two,lowerb=self.hsvTwo_lower,upperb=self.hsvTwo_upper)
            mask_object2 = cv2.bitwise_and(src1=self.CopyImage,src2=self.CopyImage,mask=mask_object2_range)
            mask_disp2 = self.cvt_cv_qt(mask_object2)
            self.disp_obj2.setPixmap(mask_disp2)
            self.disp_obj2.setScaledContents(True)
        except:
            self.Win_error.show()
            self.Win_error.Qlabel_error.setText("Please start camera before set !")

# ! Function oneclick to hsv parameter
    def GetObject_three(self):
        try:
            
            self.hsvThree_lower = np.array([self.Slider_H_min.value(), self.Slider_S_min.value(), self.Slider_V_min.value()], dtype=np.uint8)
            self.hsvThree_upper = np.array([self.Slider_H_max.value(), self.Slider_S_max.value(), self.Slider_V_max.value()], dtype=np.uint8)
            self.textEdit.append(f"Object 3 >> HSV Lower: {self.hsvThree_lower} | HSV Upper: {self.hsvThree_upper}")
            # For Debug
            #print(f"HSV Lower: {self.hsvOne_lower} | HSV Upper: {self.hsvOne_upper}")

            hsv2mask_three = cv2.cvtColor(src=self.CopyImage,code=cv2.COLOR_BGR2HSV)
            mask_object3_range = cv2.inRange(src=hsv2mask_three,lowerb=self.hsvThree_lower,upperb=self.hsvThree_upper)
            mask_object3 = cv2.bitwise_and(src1=self.CopyImage,src2=self.CopyImage,mask=mask_object3_range)
            mask_disp3 = self.cvt_cv_qt(mask_object3)
            self.disp_obj3.setPixmap(mask_disp3)
            self.disp_obj3.setScaledContents(True)
        except:
            self.Win_showError.show()
            self.Win_showError.Qlabel_error.setText("Please start camera before set !")
    
    def set_roi(self):
        if self.btn_roi_set.isChecked():
            self.btn_roi_set.setText('RESET')
            self.ckb_roi.setChecked(False)
            self.ROI_X.setEnabled(False)
            self.ROI_Y.setEnabled(False)
            self.ROI_W.setEnabled(False)
            self.ROI_H.setEnabled(False)
        else:
            self.btn_roi_set.setText('SET')
            self.ckb_roi.setChecked(True)
            self.ROI_X.setEnabled(True)
            self.ROI_Y.setEnabled(True)
            self.ROI_W.setEnabled(True)
            self.ROI_H.setEnabled(True)

    @pyqtSlot(np.ndarray)
    def opencv_emit(self, Image):

        #QPixmap format           
        original = self.cvt_cv_qt(Image)
        #Numpy Array format
        self.CopyImage =  Image[self.roi_y:self.roi_h,
                                self.roi_x:self.roi_w]
        
        """
        ในการที่จะทำ image ไปแสดงที่ Qlabel ต้องทำการ convert จาก np.array to QPixmap
        Image ที่ได้มานั้นเป็น Numpy หากนำ Image ที่ได้ไปใข้กับฟังก์ชัน original = cvt_cv_qt(Image)
        ตัวแปร original จะเป็นประเภท QtGui.QPixmap
        """

        self.disp_main.setPixmap(original)
        self.disp_main.setScaledContents(True)

    # ! Display 1   
        if self.ckb_disp1.isChecked() and self.btn_setObject1.isChecked() == False and self.track_obj1.isChecked() == False: 
            hsv_object_one_lower = np.array([self.Slider_H_min.value(), self.Slider_S_min.value(), self.Slider_V_min.value()], dtype=np.uint8)
            hsv_object_one_upper = np.array([self.Slider_H_max.value(), self.Slider_S_max.value(), self.Slider_V_max.value()], dtype=np.uint8)

            hsv_to_ObjectOne = cv2.cvtColor(src=self.CopyImage,code=cv2.COLOR_BGR2HSV)
            hsv_target_ObjectOne = cv2.inRange(src=hsv_to_ObjectOne,lowerb=hsv_object_one_lower,upperb=hsv_object_one_upper)
            hsv_one_disp = self.cvt_cv_qt(hsv_target_ObjectOne)
            self.disp_obj1.setPixmap(hsv_one_disp)
            self.disp_obj1.setScaledContents(True)

    # ! Display 2  
        if self.ckb_disp2.isChecked() and self.btn_setObject2.isChecked() == False:     
            hsv_object_two_lower = np.array([self.Slider_H_min.value(), self.Slider_S_min.value(), self.Slider_V_min.value()], dtype=np.uint8)
            hsv_object_two_upper = np.array([self.Slider_H_max.value(), self.Slider_S_max.value(), self.Slider_V_max.value()], dtype=np.uint8)
           
            hsv_to_ObjectTwo = cv2.cvtColor(src=self.CopyImage,code=cv2.COLOR_BGR2HSV)
            hsv_target_ObjectTwo = cv2.inRange(src=hsv_to_ObjectTwo,lowerb=hsv_object_two_lower,upperb=hsv_object_two_upper)
            hsv_two_disp = self.cvt_cv_qt(hsv_target_ObjectTwo)
            self.disp_obj2.setPixmap(hsv_two_disp)
            self.disp_obj2.setScaledContents(True)

        
    # ! Display 3  
        if self.ckb_disp3.isChecked() and self.btn_setObject3.isChecked() == False:     
            hsv_object_three_lower = np.array([self.Slider_H_min.value(), self.Slider_S_min.value(), self.Slider_V_min.value()], dtype=np.uint8)
            hsv_object_three_upper = np.array([self.Slider_H_max.value(), self.Slider_S_max.value(), self.Slider_V_max.value()], dtype=np.uint8)
           
            hsv_to_ObjectThree = cv2.cvtColor(src=self.CopyImage,code=cv2.COLOR_BGR2HSV)
            hsv_target_ObjectThree = cv2.inRange(src=hsv_to_ObjectThree,lowerb=hsv_object_three_lower,upperb=hsv_object_three_upper)
            hsv_three_disp = self.cvt_cv_qt(hsv_target_ObjectThree)
            self.disp_obj3.setPixmap(hsv_three_disp)
            self.disp_obj3.setScaledContents(True)

    # Display Object Tracking
        if self.track_obj1.isChecked():
            
            Track_object1 = self.Track_Function1.track_object(Image=self.CopyImage,
                                                                  HSVLower=self.hsvOne_lower,
                                                                  HSVUpper=self.hsvOne_upper,
                                                                  Color=self.RanColor1)
        # Get value object
            self.value_curr_object1 = self.Track_Function1.get_current()
            self.value_total_object1 = self.Track_Function1.get_total()

        # Get object in screen
            #self.IsObject1 = self.Track_Function1.check_object()

        # Show object value to lcdNumber
            self.lcd_curr_object1.display(self.value_curr_object1)
            self.lcd_object1.display(self.value_total_object1)

        # Convert function from Numpy to QPixmap
            cvt2Tack_1 = self.cvt_cv_qt(Track_object1)

        # Show on the main screen
            self.disp_main.setPixmap(cvt2Tack_1)
            self.disp_main.setScaledContents(True)

        if self.track_obj2.isChecked():
            Track_object2 = self.Track_Function2.track_object(Image=self.CopyImage,
                                                             HSVLower=self.hsvTwo_lower,
                                                             HSVUpper=self.hsvTwo_upper,
                                                             Color=self.RanColor2)
            
        # Get value object
            self.value_curr_object2 = self.Track_Function2.get_current()
            self.value_total_object2 = self.Track_Function2.get_total()

        # Show object value to lcdNumber
            self.lcd_curr_object2.display(self.value_curr_object2)
            self.lcd_object2.display(self.value_total_object2)

            cvt2Tack_2 = self.cvt_cv_qt(Track_object2)
            self.disp_main.setPixmap(cvt2Tack_2)
            self.disp_main.setScaledContents(True)

        
        if self.track_obj3.isChecked():
            Track_object3 = self.Track_Function3.track_object(Image=self.CopyImage,
                                                             HSVLower=self.hsvThree_lower,
                                                             HSVUpper=self.hsvThree_upper,
                                                             Color=self.RanColor3)
            
        # Get value object
            self.value_curr_object3 = self.Track_Function3.get_current()
            self.value_total_object3 = self.Track_Function3.get_total()

        # Show object value to lcdNumber
            self.lcd_curr_object3.display(self.value_curr_object3)
            self.lcd_object3.display(self.value_total_object3)
            
            cvt2Tack_3 = self.cvt_cv_qt(Track_object3)
            self.disp_main.setPixmap(cvt2Tack_3)
            self.disp_main.setScaledContents(True)
    
    def get_ROIX(self,x):
        self.roi_x = x
    
    def get_ROIY(self,y):
        self.roi_y = y
    
    def get_ROIW(self,w):
        self.roi_w = w
    
    def get_ROIH(self,h):
        self.roi_h = h
   
    def cvt_cv_qt(self, Image):
        offset = 5
        rgb_img = cv2.cvtColor(src=Image,code=cv2.COLOR_BGR2RGB)
        if self.ckb_roi.isChecked():
            rgb_img = cv2.rectangle(rgb_img,
                                         pt1=(self.roi_x,self.roi_y),
                                         pt2=(self.roi_w,self.roi_h),
                                         color=(0,255,255),
                                         thickness=2)

        h,w,ch = rgb_img.shape
        bytes_per_line = ch * w
        cvt2QtFormat = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(cvt2QtFormat)
        if self.track_obj1.isChecked() or self.track_obj2.isChecked() or self.track_obj3.isChecked():
            pixmap = QPixmap.fromImage(cvt2QtFormat)
            painter = QPainter(pixmap)
            pen = QPen(Qt.red,3)
            painter.setPen(pen)
            painter.drawRect(self.roi_x-(self.roi_x-offset),
                             self.roi_y-(self.roi_y-offset),
                             self.roi_w-30,
                             self.roi_h-30
                             )

        return pixmap #QPixmap.fromImage(cvt2QtFormat)
#----------------------------------------------------------------------------------------------------

    def StartWebCam(self,pin):
        try:
            self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Start Webcam ({self.camlist.currentText()})")
            self.btn_stop.setEnabled(True)
            self.btn_start.setEnabled(False)

            global camIndex
            camIndex = self.camlist.currentIndex()
        
        # Opencv QThread
            self.Worker1_Opencv = ThreadClass()
            self.Worker1_Opencv.ImageUpdate.connect(self.opencv_emit)
            self.Worker1_Opencv.FPS.connect(self.get_FPS)
            self.Worker1_Opencv.start()
        

        except Exception as error :
            pass
    
    def StopWebcam(self,pin):
        self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Stop Webcam ({self.camlist.currentText()})")
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

        # Set Icon back to stop state
        self.Motor_status.setPixmap(QPixmap('Icons/MotorOFF.png'))
        self.Worker1_Opencv.stop()
        self.motor_on.stop()

    def Close_software(self):
        self.Worker1_Opencv.stop()
        self.resource_usage.stop()
        sys.exit(app.exec_())

    def Motor_on(self):
        if self.btn_motor.isChecked():
            self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Motor ON")
            self.btn_motor.setText('Motor OFF')
            #self.motor_on = QTimer(self, interval=1000)
            #self.motor_on.timeout.connect(self.Running_lamp)
            #self.motor_on.timeout.connect(self.Motor_state)
            self.motor_on.start()
        else:
            self.textEdit.append(f"{self.DateTime.toString('d MMMM yy hh:mm:ss')}: Motor OFF")
            self.motor_on.stop()
            self.btn_motor.setText('Motor ON')

# Motor, Tower Lamp Status    
    def Motor_state(self):
        if self.flag_motor: self.Motor_status.setPixmap(QPixmap('Icons/MotorON.png'))
        else : self.Motor_status.setPixmap(QPixmap('Icons/MotorOFF.png'))
        self.flag_motor = not self.flag_motor
        
    
    def Ready_lamp(self):
        if self.Status_lamp[0]: self.Qlabel_greenlight.setStyleSheet("background-color: rgb(85, 255, 0); border-radius:30px")
        else : self.Qlabel_greenlight.setStyleSheet("background-color: rgb(184, 230, 191); border-radius:30px")
        self.Status_lamp[0] = not self.Status_lamp[0]
        #IO.output(IO_OUTPUT[0], self.Status_lamp[0])

    
    def Running_lamp(self):
        if self.Status_lamp[1]: self.Qlabel_yellowlight.setStyleSheet("background-color: rgb(255, 195, 0); border-radius:30px")
        else : self.Qlabel_yellowlight.setStyleSheet("background-color: rgb(242, 214, 117); border-radius:30px")
        self.Status_lamp[1] = not self.Status_lamp[1]
        #IO.output(IO_OUTPUT[1], self.Status_lamp[1])
    
    def Alarm_lamp(self):
        if self.Status_lamp[2]: self.Qlabel_yellowlight.setStyleSheet("background-color: rgb(255, 0, 0); border-radius:30px")
        else : self.Qlabel_yellowlight.setStyleSheet("background-color:  rgb(255, 171, 175); border-radius:30px")
        self.Status_lamp[2] = not self.Status_lamp[2]


if __name__ == "__main__":
  
# IO Board
    IO_INPUT = [7,11,13,15]
    IO_OUTPUT = [12,16,18,22,32,36,38,40]
    #IO.setwarnings(False)
    #IO.setmode(IO.BOARD)
    
    for input_pins in IO_INPUT:
        pass#IO.setup(input_pins, IO.IN)
        
    for output_pins in IO_OUTPUT:
        pass#IO.setup(output_pins, IO.OUT, initial=IO.HIGH)
    
    app = QApplication([])
    window = MainWindow()
    #IO.add_event_detect(IO_INPUT[0],IO.FALLING,window.StartWebCam,500)
    #IO.add_event_detect(IO_INPUT[1],IO.FALLING,window.StopWebcam,500)
    window.show()
    app.exec_()

    #IO.cleanup()
