import Tkinter
import tkMessageBox
import data

class App:
    def __init__(self):
        self.top = Tkinter.Tk()
        self.B1 = Tkinter.Button(self.top, text ="Start All", command = self.start_all)
        self.B1.pack()
        self.B2 = Tkinter.Button(self.top, text ="MoveStraight", command = self.move_straight)
        self.B2.pack()
        self.B3 = Tkinter.Button(self.top, text ="Turn", command = self.turn)
        self.B3.pack()
        self.top.mainloop()      
    def turn():
        tkMessageBox.showinfo( "Hello Python", "Turning")
        pass

    def start_all():
        tkMessageBox.showinfo( "Hello Python", "Start all")
        pass

    def move_straight():
        tkMessageBox.showinfo( "Hello Python", "Move Straight")
        pass

if __name__ == "__main__":
    myapp = App

