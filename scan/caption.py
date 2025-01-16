from tkinter import *
from tkinter import ttk
import time

def get_text_from_file(file_path):
    with open(file_path, "r") as file:
        return file.read().strip()

def display_text():
    text = get_text_from_file("Demo_state.txt")
    label.config(text=text)
    root.after(1000, display_text)


root = Tk()
root.geometry("1920x1080")


label = ttk.Label(root, text="", font=('Arial', 80))
label.pack(pady=20)


display_text()


quit_button = ttk.Button(root, text="Quit", command=root.destroy)
quit_button.pack()


root.mainloop()
