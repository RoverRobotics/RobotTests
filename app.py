import Tkinter
from matplotlib.pyplot import text
import tkMessageBox
from controller import Controller
class App:
    def __init__(self):
        self.mycontroller = Controller()
        # self.mycontroller.set_ip("192.168.1.1")
        # self.mycontroller.set_port(80)
        self.top = Tkinter.Tk()
        self.lb1 = Tkinter.Label(self.top,text="Linear Speed (m/s)")
        self.lb1.pack()
        self.Tb1 =Tkinter.Text(self.top, height=10)
        self.Tb1.insert(Tkinter.END, '1')
        self.Tb1.pack()
        self.lb1 = Tkinter.Label(self.top,text="Angular  (r/s)")
        self.lb1.pack()
        self.Tb2 =Tkinter.Text(self.top, height=10)
        self.Tb2.insert(Tkinter.END, '1')
        self.Tb2.pack()
        self.B1 = Tkinter.Button(self.top, text ="Start All", command = self.start_all)
        self.B1.pack()
        self.B2 = Tkinter.Button(self.top, text ="MoveStraight", command = self.move_straight)
        self.B2.pack()
        self.B3 = Tkinter.Button(self.top, text ="Turn", command = self.turn)
        self.B3.pack()
        self.top.mainloop()    
    def turn(self):
        tkMessageBox.showinfo( "Hello Python", "Turning")
        # self.mycontroller.set_ip("192.168.1.1")
        self.mycontroller.set_port(80)
        pass

    def start_all(self):
        tkMessageBox.showinfo( "Hello Python", "Start all")
        pass

    def move_straight(self):
        self.mycontroller.set_linear_velocity(int(self.Tb1.get("1.0","end")))
        error = self.mycontroller.start_move_straight(int(self.Tb2.get("1.0","end")))
        if(error <=0):
            tkMessageBox.showinfo( "Info", "Ran Move Straight")
        else:
            tkMessageBox.showinfo( "Info", "Error")
        

if __name__ == "__main__":
    myapp = App()

