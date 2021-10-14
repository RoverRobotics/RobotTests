import Tkinter
import tkMessageBox
from controller import Controller
class App:
    def __init__(self):
        self.mycontroller = Controller()
        # self.mycontroller.set_ip("192.168.1.1")
        # self.mycontroller.set_port(80)
        self.top = Tkinter.Tk()
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
        self.mycontroller.set_linear_velocity(.75)
        self.mycontroller.start_move_straight(5)
        tkMessageBox.showinfo( "Hello Python", "Move Straight")
        pass

if __name__ == "__main__":
    myapp = App()

