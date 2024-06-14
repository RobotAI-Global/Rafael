import tkinter as tk
from tkinter import ttk
import json
import uuid

# Show yaml in tkinker
#from pathlib import Path
from tkinter import Frame, Canvas, Scrollbar
import numpy as np
# Setup Data
Data = {
    "firstName": "John",
    "lastName": "Smith",
    "gender": "male",
    "age": 32,
    "address": {
        "streetAddress": "21 2nd Street",
        "city": "New York",
        "state": "NY",
        "postalCode": "10021"},
    "phoneNumbers": [
        {"type": "home", "number": "212 555-1234" },
        {"type": "fax",
         "number": "646 555-4567",
         "alphabet": [
            "abc",
            "def",
            "ghi"]
        }
    ]}
        
# opt_name: (from_, to, increment)
IntOptions = {
    'age': (1.0, 200.0, 1.0),
}        

def close_ed(parent, edwin):
    parent.focus_set()
    edwin.destroy()


def set_cell(edwin, w, tvar):
    value = tvar.get()
    w.item(w.focus(), values=(value,))
    close_ed(w, edwin)


def edit_cell(e):
    w = e.widget
    if w and len(w.item(w.focus(), 'values')) > 0:
        edwin = tk.Toplevel(e.widget)
        edwin.protocol("WM_DELETE_WINDOW", lambda: close_ed(w, edwin))
        edwin.wait_visibility()
        edwin.grab_set()
        edwin.overrideredirect(1)
        opt_name = w.focus()
        (x, y, width, height) = w.bbox(opt_name, 'Values')
        edwin.geometry('%dx%d+%d+%d' % (width, height, x/4, y))
        value = w.item(opt_name, 'values')[0]
        tvar = tk.StringVar()
        tvar.set(str(value))
        ed = None
        if opt_name in IntOptions:
            constraints = IntOptions[opt_name]
            ed = tk.Spinbox(edwin, from_=constraints[0], to=constraints[1],
                increment=constraints[2], textvariable=tvar)
        else:
            ed = tk.Entry(edwin, textvariable=tvar)
        if ed:
            ed.config(background='LightYellow')
            #ed.grid(column=0, row=0, sticky=(tk.N, tk.S, tk.W, tk.E))
            ed.pack()
            ed.focus_set()
        edwin.bind('<Return>', lambda e: set_cell(edwin, w, tvar))
        edwin.bind('<Escape>', lambda e: close_ed(w, edwin)) 

def JSONTree(Tree, Parent, Dictionary):
    for key in Dictionary :
        uid = uuid.uuid4()
        if isinstance(Dictionary[key], dict):
            Tree.insert(Parent, 'end', uid, text=key)
            JSONTree(Tree, uid, Dictionary[key])
        elif isinstance(Dictionary[key], list):
            #print(key)
            Tree.insert(Parent, 'end', uid, text=key + '[]')
            JSONTree(Tree,
                     uid,
                     dict([(i, x) for i, x in enumerate(Dictionary[key])]))
        else:
            value = Dictionary[key]
            if isinstance(value, str) or isinstance(value, str):
                value = value.replace(' ', '_')
                Tree.insert(Parent, 'end', uid, text=key, value=value)




class JsonEditor(object):

    def __init__(self, fname_ext = None):
        self.name = None
        self.root = None
        
        self.frame = None  # for scrolling
        self.lbl = None
        self.txt = None    # list of entries
        self.val = None    # holds the values
        self.types = None  # holds the types 
        self.data_changed = False
        self.current_rendered_window = {'main_frame':0, 'main_canvas':0, 'horizontal_scrollbar':0, 'main_scrollbar':0,'secondary_frame':0}

        #self.config = Config().load_yaml()
        self.fname = r'D:\RobotAI\Customers\Primatic\Rafael\main_session.json' if fname_ext is None else fname_ext
        self.data  = self.load_json()

    def show_scrolling(self):
        self.scrolled_window()
        #self.frame   = self.current_rendered_window["secondary_frame"]
        #self.show_entries()
        self.root.mainloop()



    def create_window(self):
        self.window = tk.Tk()
        self.window.title("Config File Editor")
        self.window.geometry('800x800')
        #self.window.iconbitmap(iconFilePath)
        
    def scrolled_window(self):
        # Create a main frame with a canvas so that it's possible use a scroll bar
        #self.window.geometry('800x800')
        
        # Setup the root UI
        self.root = tk.Tk()
        self.root.title("Session File Editor")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)  
        #self.root.geometry('800x800')
        #self.window.iconbitmap(iconFilePath)        
        
        # Setup the Frames
        TreeFrame = ttk.Frame(self.root, padding="3")
        TreeFrame.grid(row=0, column=0, sticky=tk.NSEW)
    
        # Setup the Tree
        tree = ttk.Treeview(TreeFrame, columns=('Values'))
        tree.column('Values', width=100, anchor='center')
        tree.heading('Values', text='Values')
        tree.bind('<Double-1>', edit_cell)
        tree.bind('<Return>', edit_cell)
        JSONTree(tree, '', self.data)
        tree.pack(fill=tk.BOTH, expand=1)
    
        # Limit windows minimum dimensions
        self.root.update_idletasks()
        self.root.minsize(self.root.winfo_reqwidth(), self.root.winfo_reqheight())
        
    def load_json(self) -> dict:
        #return yaml.load(self.document_yaml)
        filename = self.fname
        with open(filename, 'rb') as f:
            json_data = json.load(f)  
        return json_data
    
    def save_json(self, json_data):
        filename = self.fname
        with open(filename, "w") as outfile: 
            #json.dump(json_data, outfile) 
            json.dump(json_data, outfile, indent=4) 

    def save_callback(self):
                  
        self.update_entries()
        print('save is succesful')
        self.finish_gui()
        return True
        
    def cancel_callback(self):       
        print('cancel is pressed')  
        self.finish_gui()



    def finish_gui(self):
        self.window.destroy()
        self.window.quit()

#my_window = Window()
#my_window.show()
#my_window.show_scrolling()

#class JSONViewerApp:
#    def __init__(self, root):
#        self.root = root
#        self.root.title("JSON Viewer")
#
#        # Create a Frame for displaying JSON data
#        self.frame = ttk.Frame(root)
#        self.frame.pack(padx=20, pady=20, fill=tk.BOTH, expand=True)
#
#        # Create a Text widget for displaying JSON content
#        self.text_widget = tk.Text(self.frame, wrap=tk.WORD)
#        self.text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
#
#        # Create a Scrollbar for the Text widget
#        self.scrollbar = ttk.Scrollbar(self.frame, command=self.text_widget.yview)
#        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
#        self.text_widget.config(yscrollcommand=self.scrollbar.set)
#
#        # Load and display JSON data
#        self.load_json_data(fname)  # Replace with your JSON file path
#
#    def load_json_data(self, json_file):
#        try:
#            # Load JSON data from the file
#            with open(json_file, "r") as file:
#                data = json.load(file)
#
#            # Format JSON data as a string and display it in the Text widget
#            formatted_json = json.dumps(data, indent=4)
#            self.text_widget.insert(tk.END, formatted_json)
#
#            # Make the Text widget read-only
#            self.text_widget.config(state=tk.DISABLED)
#
#        except FileNotFoundError:
#            self.text_widget.insert(tk.END, "File not found.")
#            self.text_widget.config(state=tk.DISABLED)

#if __name__ == "__main__":
#    root = tk.Tk()
#    app = JSONViewerApp(root)
#    root.mainloop()


#from pprint import pprint as pprint




if __name__ == "__main__" :
    # Setup the root UI
#    root = tk.Tk()
#    root.title("JSON editor")
#    root.columnconfigure(0, weight=1)
#    root.rowconfigure(0, weight=1)
#
#
#
#    # Setup the Frames
#    TreeFrame = ttk.Frame(root, padding="3")
#    TreeFrame.grid(row=0, column=0, sticky=tk.NSEW)
#
#    # Setup the Tree
#    tree = ttk.Treeview(TreeFrame, columns=('Values'))
#    tree.column('Values', width=100, anchor='center')
#    tree.heading('Values', text='Values')
#    tree.bind('<Double-1>', edit_cell)
#    tree.bind('<Return>', edit_cell)
#    JSONTree(tree, '', Data)
#    tree.pack(fill=tk.BOTH, expand=1)
#
#    # Limit windows minimum dimensions
#    root.update_idletasks()
#    root.minsize(root.winfo_reqwidth(), root.winfo_reqheight())
#    root.mainloop()
    my_window = JsonEditor()
#my_window.show()
    my_window.show_scrolling()