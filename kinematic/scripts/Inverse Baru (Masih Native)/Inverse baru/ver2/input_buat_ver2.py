from TestWawancara import kinematika_robot

robot = kinematika_robot()

vx = float(input("Masukkan nilai Vx: "))
vy = float(input("Masukkan nilai Vy: "))
vtheta = float(input("Masukkan nilai Vtheta: "))

w1, w2, w3, w4 = robot.perumusan(vx, vy, vtheta)

print(f"kecepatan pada roda pertama adalah  : {w1}")
print(f"kecepatan pada roda kedua adalah    : {w2}")
print(f"kecepatan pada roda ketiga adalah   : {w3}")
print(f"kecepatan pada roda keempat adalah  : {w4}")