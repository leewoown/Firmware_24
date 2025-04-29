from TSMaster import *
import sys
blacklist = ["tkinter"] # modules such as tkinter will release GIL by itself, which is not allowed in TSMster Toolbox
for mod in blacklist:
    if sys.modules.get(mod):
        tmp_import = __import__ (mod)
        sys.modules[mod] = None

# Auto Generated Python Code, do not modify START [UI] --------------
class leewo_2025_04_28_23_12_58(frmTSForm):
    def __init__(self):
        # set type name internally
        self.ExternalTypeName = type(self).__name__
        if hasattr(self, "on_before_ui_creation"):
            method = getattr(self, "on_before_ui_creation")
            method()
        try:
            # Form properties
            if not self.IsConfigurationLoaded:
                self.Left = 249
                self.Top = 150
                self.Caption = 'leewo_2025_04_28_23_12_58'
                self.ClientHeight = 502
                self.ClientWidth = 786
            self.Color = clBtnFace
            self.DoubleBuffered = True
            self.Font.Charset = DEFAULT_CHARSET
            self.Font.Color = clWindowText
            self.Font.Height = -12
            self.Font.Name = 'Segoe UI'
            self.Font.Style = []
            self.KeyPreview = True
            self.Position = "poDesigned"
            self.Visible = True
            self.TextHeight = 15
            # Create control: Label1 = Label(self)
            self.Label1 = Label(self)
            self.Label1.Name = "Label1"
            self.Label1.Parent = self
            self.Label1.Left = 336
            self.Label1.Top = 224
            self.Label1.Width = 34
            self.Label1.Height = 15
            self.Label1.Cursor = crArrow
            self.Label1.Caption = 'Label1'
            # Create control: Button1 = Button(self)
            self.Button1 = Button(self)
            self.Button1.Name = "Button1"
            self.Button1.Parent = self
            self.Button1.Left = 152
            self.Button1.Top = 104
            self.Button1.Width = 75
            self.Button1.Height = 25
            self.Button1.Cursor = crArrow
            self.Button1.CustomHint = app.ui_get_default_ballon_hint()
            self.Button1.Caption = 'Button1'
            self.Button1.Images = app.get_system_imagelist_16px()
            self.Button1.TabOrder = 0
            # Create control: TrackBar1 = TrackBar(self)
            self.TrackBar1 = TrackBar(self)
            self.TrackBar1.Name = "TrackBar1"
            self.TrackBar1.Parent = self
            self.TrackBar1.Left = 112
            self.TrackBar1.Top = 184
            self.TrackBar1.Width = 200
            self.TrackBar1.Height = 48
            self.TrackBar1.Cursor = crArrow
            self.TrackBar1.TabOrder = 9
            # Create control: RadioGroup1 = RadioGroup(self)
            self.RadioGroup1 = RadioGroup(self)
            self.RadioGroup1.Name = "RadioGroup1"
            self.RadioGroup1.Parent = self
            self.RadioGroup1.Left = 520
            self.RadioGroup1.Top = 216
            self.RadioGroup1.Width = 185
            self.RadioGroup1.Height = 105
            self.RadioGroup1.Cursor = crArrow
            self.RadioGroup1.Caption = 'RadioGroup1'
            self.RadioGroup1.TabOrder = 10
            # Create control: CheckBox1 = CheckBox(self)
            self.CheckBox1 = CheckBox(self)
            self.CheckBox1.Name = "CheckBox1"
            self.CheckBox1.Parent = self
            self.CheckBox1.Left = 400
            self.CheckBox1.Top = 120
            self.CheckBox1.Width = 97
            self.CheckBox1.Height = 17
            self.CheckBox1.Cursor = crArrow
            self.CheckBox1.Caption = 'CheckBox1'
            self.CheckBox1.TabOrder = 11
        finally:
            # End UI auto creation
            self.EndUIAutoCreation()
# Auto Generated Python Code, do not modify END [UI] ----------------
        # your init code starts here...
        
# Auto Generated Python Code, do not modify START [MAIN] ------------
if __name__ == "__main__":
    try:
        leewo_2025_04_28_23_12_58().Show()
        Application.Run()
    except SystemExit:
        pass
# Auto Generated Python Code, do not modify END [MAIN] --------------
