import cv2



def line(frameSet, x_awal, y_awal, x_akhir, y_akhir, colour_name, thickness=1):
    """
    MENAMBAHKAN GARIS PADA DISPLAY
    :PILIHAN WARNA(HIJAU, BIRU, MERAH, HITAM, PUTIH)
    :WAJIB CAPS
    """
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }
    
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.line(frameSet, (x_awal, y_awal), (x_akhir, y_akhir), colour, thickness)
    return frameSet

def circle(frameSet, center_x, center_y, radius, coulour_name, thickness = 1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }
    colour = WARNA.get(coulour_name, (0, 255, 0)) 
    cv2.circle(frameSet, (center_x, center_y),radius, colour, thickness)
    return frameSet

def rect(frameSet, x_awal, y_awal, x_akhir, y_akhir, colour_name, thickness=1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }
    
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.rectangle(frameSet, (x_awal, y_awal), (x_akhir, y_akhir), colour, thickness)
    return frameSet

def crossPlus(frameSet, W_cam = 640, H_cam = 480, colour_name = "HIJAU", thickness = 1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }   
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.line(frameSet, (W_cam//2, 0), (W_cam//2, 480), colour, thickness) #VERTICAL
    cv2.line(frameSet, (0, H_cam//2), (640, H_cam//2), colour, thickness) #HORIZONTAL
    return frameSet

def crossFront(frameSet, W_cam = 640, colour_name = "HIJAU", H_adjust = 360, thickness = 1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }   
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.line(frameSet, (W_cam//2, 0), (W_cam//2, 480), colour, thickness) #VERTICAL
    cv2.line(frameSet, (0, H_adjust), (640, H_adjust), colour, thickness) #HORIZONTAL
    return frameSet

def crossAdj(frameSet, W_cam = 640, H_cam = 480,offset_W = 0,offset_H = 0, colour_name = "HIJAU", thickness = 1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }   
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.line(frameSet, ((W_cam//2)+offset_W, 0), ((W_cam//2)+offset_W, 480), colour, thickness) #VERTICAL
    cv2.line(frameSet, (0, (H_cam//2)+offset_H), (640, (H_cam//2)+offset_H), colour, thickness) #HORIZONTAL
    return frameSet

def text(frameSet, string, posX, posY, fontType, fontSize, colour_name= "HIJAU", fontThick= 1):
    if frameSet is None:
        print("Error: Frame tidak valid.")
        return None
    FONT = {
        "SIMPLEX": cv2.FONT_HERSHEY_SIMPLEX,
        "PLAIN": cv2.FONT_HERSHEY_PLAIN,
        "DUPLEX": cv2.FONT_HERSHEY_DUPLEX,
        "COMPLEX": cv2.FONT_HERSHEY_COMPLEX,
        "TRIPLEX": cv2.FONT_HERSHEY_TRIPLEX,
        "COMPLEX_SMALL": cv2.FONT_HERSHEY_COMPLEX_SMALL,
        "SCRIPT_SIMPLEX": cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,
        "SCRIPT_COMPLEX": cv2.FONT_HERSHEY_SCRIPT_COMPLEX
    }
    font = FONT.get(fontType, cv2.FONT_HERSHEY_SIMPLEX)
    
    WARNA = {
        "HIJAU": (0, 255, 0),
        "BIRU": (255, 0, 0),
        "MERAH": (0, 0, 255),
        "HITAM": (0, 0, 0),
        "PUTIH": (255, 255, 255)
        }   
    colour = WARNA.get(colour_name, (0, 255, 0))
    cv2.putText(frameSet, string,(posX,posY), font, fontSize, colour, fontThick)
    return frameSet