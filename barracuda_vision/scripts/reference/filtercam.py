import cv2

class filter_cam:
    def __init__(self):
        pass
    def blurFilter(self, frameSet, blurVal):
        """
        Menerapkan Effect Blur Filter
        :blurval mengatur nilai blur nilai harus ganjil 1, 3, 5, 7, ...
        """
        if frameSet is None:
            print("Error: Frame tidak valid.")
            return None
        blurFrame = cv2.GaussianBlur(frameSet, (blurVal, blurVal), 0)
        return blurFrame
    
    def hsvFilter(self, frameSet):
        """
        Menerapkan Effect HSV
        """
        if frameSet is None:
            print("Error: Frame tidak valid.")
            return None
        hsv = cv2.cvtColor(frameSet, cv2.COLOR_BGR2HSV)
        return hsv
    
    def grayFilter(self, frameSet):
        """
        Menerapkan Effect Abu Abu
        """
        if frameSet is None:
            print("Error: Frame tidak valid.")
            return None
        gray = cv2.cvtColor(frameSet, cv2.COLOR_BGR2GRAY)
        return gray   
    
    def mask(self, frameSet, H_min, S_min, V_min, H_max, S_max, V_max):
        """
        Menerapkan Effect Masking Baseon HSV
        :H = Hue
        :S = Saturation
        :V = Value
        """
        if frameSet is None:
            print("Error: Frame tidak valid.")
            return None
        if len(frameSet.shape) != 3 or frameSet.shape[2] != 3:
            print("Error: Frame harus dalam format BGR atau HSV.")
            return None
        masking = cv2.inRange(self.hsvFilter(frameSet),(H_min, S_min, V_min),(H_max, S_max, V_max))
        return masking
    
    def contourDetect(self, frameSet, indexRet = 0, indexAprx = 0):
        """
        Mencari Contour Object
        :IndexRet mengatur mode retrieves 0-4(EXTERNAL, LIST, CCOMP, TREE, FLOODFILL)
        :IndexAprx mengatur mode approx 0-3 (NONE,SIMPLE,TC89-LI,TC89-KCOS)(Penyederhanaan titik contour)
        """        
        if frameSet is None:
            print("Error: Frame tidak valid.")
            return None
        # RET MODE SELECTOR
        ret_modes = {
            0: cv2.RETR_EXTERNAL,
            1: cv2.RETR_LIST,
            2: cv2.RETR_CCOMP,
            3: cv2.RETR_TREE,
            4: cv2.RETR_FLOODFILL
        }
        RetMode = ret_modes.get(indexRet, cv2.RETR_EXTERNAL)
        
        # APRX MODE SELECTOR
        aprx_modes = {
            0: cv2.CHAIN_APPROX_NONE,
            1: cv2.CHAIN_APPROX_SIMPLE,
            2: cv2.CHAIN_APPROX_TC89_L1,
            3: cv2.CHAIN_APPROX_TC89_KCOS
        }       
        AprxMode = aprx_modes.get(indexAprx, cv2.CHAIN_APPROX_SIMPLE)

        contour, hierarcy = cv2.findContours(frameSet,RetMode, AprxMode)
        return contour,hierarcy
    
    def cannyFilter(self, frameSet, threshold1, threshold2,apertureSize = 3):
        """
        Mencari Tepi Objek
        :threshold1
        :threshold2
        """  
        if frameSet is None:
            print("Error: Frame tidak valid.")
            return None
        edges = cv2.Canny(frameSet, threshold1, threshold2, apertureSize)
        return edges
    
    def flipFrame(self, frameSet, flipCode = 0):
        """
        Memutar balik Frame
        :flipCode
        :0  = flip by vertical
        :1  = flip by horizontal
        ;-1 = flip both (vertical & horizontal)
        """
        if frameSet is None:
            print("Error: Frame tidak valid.")
            return None
        flip = cv2.flip(frameSet, flipCode)
        return flip