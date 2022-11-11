import sys
input=sys.stdin.readline

data=[]

_max=0.0

for _ in range(480):
    row=list(map(float,input().split(',')[:-1]))
    data.append(row)
    _max=max(_max,max(row))

data=list(map(lambda row:list(map(lambda x:x/_max*255,row)),data))

import tkinter as tk

root=tk.Tk()
root.geometry("1080x1080")

canvas = tk.Canvas(root, width = 1080, height = 1080)
canvas.pack()

for y,row in enumerate(data):
    for x,d in enumerate(row):
        color=format(int(d),'02x')
        color='#'+color+color+color
        canvas.create_rectangle(x*2,y*2,x*2+2,y*2+2,fill=color)

root.mainloop()