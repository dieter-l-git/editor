import re
import datetime
import threading
import queue
import serial.tools.list_ports
from builtins import str as text
from arduino import builder
import util.paths as paths

import wx
import wx.stc as stc

import time
import os
import platform
import json
import time
import glob

# -------------------------------------------------------------------------------
#                            Arduino Upload Dialog
# -------------------------------------------------------------------------------

class ArduinoUploadDialog(wx.Dialog):
    """Dialog to configure upload parameters"""
    BUILD_OPTIONS = [
            (_("Use build cache"), builder.BuildCacheOption.USE_CACHE),
            (_("Clean build cache"), builder.BuildCacheOption.CLEAN_BUILD),
            (_("Clean build cache, upgrade core"), builder.BuildCacheOption.UPGRADE_CORE),
            (_("Clean build cache, upgrade libraries"), builder.BuildCacheOption.UPGRADE_LIBS),
            (_("Clean build cache, reinstall libraries"), builder.BuildCacheOption.CLEAN_LIBS),
            (_("Mr. Proper (Clean, reinstall core, board and libraries)"), builder.BuildCacheOption.MR_PROPER)
        ]


    def __init__(self, parent, st_code, arduino_ext, md5, project_controller):
        """
        Constructor
        @param parent: Parent wx.Window of dialog for modal
        @param st_code: Compiled PLC program as ST code.
        """
        self.plc_program = st_code
        self.arduino_sketch = arduino_ext
        self.definitions = []
        self.md5 = md5
        self.settings = {}
        current_dir = paths.AbsDir(__file__)
        self.com_port_combo_choices = {}
        self.project_controller = project_controller
        self.settings = self.project_controller.GetArduinoSettings()
        self.active_build_option = builder.BuildCacheOption.USE_CACHE  # Set "Use build cache" as default
        self.workaround_macos = platform.system() == 'Darwin'

        wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = _('Transfer Program to PLC'), pos = wx.DefaultPosition, style = wx.DEFAULT_DIALOG_STYLE )

        # load Hals automatically and initialize the board_type_comboChoices
        self.loadHals()
        self.updateInstalledBoards()
        board_type_comboChoices = []
        for board in self.hals:
            board_name = ""
            if self.hals[board]['version'] == "0":
                board_name = board + ' [' + _('NOT INSTALLED') + ']'
            else:
                board_name = board + ' [' + self.hals[board]['version'] + ']'

            board_type_comboChoices.append(board_name)
        board_type_comboChoices.sort()

        self.SetSizeHints(wx.Size(-1,-1), wx.DefaultSize)

        main_sizer = wx.BoxSizer(wx.VERTICAL)

        # Create a panel for Board Type and COM Port
        top_panel = wx.Panel(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL)
        top_sizer = wx.GridBagSizer(vgap=5, hgap=5)

        # Set a minimum width for labels, matching the Listbook graphics width
        label_width = 100  # Adjust this value to match your Listbook graphics width

        # Board Type
        self.m_staticText1 = wx.StaticText(top_panel, wx.ID_ANY, _('Board Type'), wx.DefaultPosition, wx.Size(label_width, -1), 0)
        self.m_staticText1.Wrap(-1)
        top_sizer.Add(self.m_staticText1, pos=(0,0), flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT, border=5)

        self.board_type_combo = wx.ComboBox(top_panel, wx.ID_ANY, "Arduino Uno", wx.DefaultPosition, wx.Size(-1,-1), board_type_comboChoices, 0)
        top_sizer.Add(self.board_type_combo, pos=(0,1), flag=wx.ALL | wx.EXPAND, border=0)

        self.m_staticline1 = wx.StaticLine(top_panel, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        top_sizer.Add(self.m_staticline1, pos=(1,0), span=(1,3), flag=wx.EXPAND, border=5)

        top_sizer.AddGrowableCol(1, 1)  # Make the middle column (index 1) growable
        top_panel.SetSizer(top_sizer)
        top_sizer.Fit(top_panel)
        main_sizer.Add(top_panel, 0, wx.EXPAND | wx.ALL, 5)

        self.m_listbook2 = wx.Listbook(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LB_LEFT)
        m_listbook2ImageSize = wx.Size(100,100)
        m_listbook2Index = 0
        m_listbook2Images = wx.ImageList(m_listbook2ImageSize.GetWidth(), m_listbook2ImageSize.GetHeight())

        self.m_listbook2.AssignImageList(m_listbook2Images)
        self.m_panel5 = wx.Panel(self.m_listbook2, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL)

        # Create a vertical sizer as container for the widgets below
        bSizer21 = wx.BoxSizer(wx.VERTICAL)

        # Create a GridBagSizer for the COM port and control elements
        gbs = wx.GridBagSizer(vgap=5, hgap=5)

        # COM Port row
        com_label = wx.StaticText(self.m_panel5, wx.ID_ANY, _('COM Port'), wx.DefaultPosition, wx.Size(100, -1), 0)
        com_label.Wrap(-1)
        gbs.Add(com_label, pos=(0,0), flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL | wx.ALIGN_RIGHT, border=5)

        self.com_port_combo = wx.ComboBox(self.m_panel5, wx.ID_ANY, "COM1", wx.DefaultPosition, wx.Size(-1,-1), [""], 0)
        gbs.Add(self.com_port_combo, pos=(0,1), flag=wx.ALL | wx.EXPAND, border=5)

        button_size = self.com_port_combo.GetSize().GetHeight()
        self.reload_button = wx.Button(self.m_panel5, wx.ID_ANY, "\u21BB", wx.DefaultPosition, size=(button_size, button_size), style=wx.BU_EXACTFIT)
        self.reload_button.SetToolTip(_('Reload COM port list'))
        gbs.Add(self.reload_button, pos=(0,2), flag=wx.ALL | wx.ALIGN_RIGHT, border=5)

        # Bind events for Comboboxes and Button
        self.board_type_combo.Bind(wx.EVT_COMBOBOX, self.onBoardChange)
        self.com_port_combo.Bind(wx.EVT_COMBOBOX_DROPDOWN, self.reloadComboChoices)
        self.reload_button.Bind(wx.EVT_BUTTON, self.reloadComboChoices)

        # Create compile only checkbox
        self.check_compile = wx.CheckBox(self.m_panel5, wx.ID_ANY, _('Compile Only'), wx.DefaultPosition, wx.DefaultSize, 0)
        self.check_compile.Bind(wx.EVT_CHECKBOX, self.onUIChange)
        # Add to horizontal sizer, aligned left
        gbs.Add(self.check_compile, pos=(1,0), flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL, border=5)

        # Create a combobox for build options selection with read-only style
        self.build_options = wx.ComboBox(
            self.m_panel5,
            wx.ID_ANY,
            choices=[option[0] for option in self.BUILD_OPTIONS],
            style=wx.CB_DROPDOWN | wx.CB_READONLY
        )
        # Set default selection to first item
        self.build_options.SetSelection(0)
        # Bind the selection change event
        self.build_options.Bind(wx.EVT_COMBOBOX, self.onBuildCacheOptionChange)
        # Add to horizontal sizer with right padding
        gbs.Add(self.build_options, pos=(1,1), flag=wx.ALL | wx.EXPAND, border=5)

        # Make the middle column growable
        gbs.AddGrowableCol(1)

        bSizer21.Add(gbs, 0, wx.EXPAND | wx.ALL, 5)

        self.m_staticline2 = wx.StaticLine(self.m_panel5, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        bSizer21.Add(self.m_staticline2, 0, wx.EXPAND |wx.ALL, 5)

        self.m_staticText3 = wx.StaticText(self.m_panel5, wx.ID_ANY, _('Compilation output'), wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText3.Wrap(-1)
        bSizer21.Add(self.m_staticText3, 0, wx.ALL, 5)

        # build output widget
        self.output_text = stc.StyledTextCtrl(self.m_panel5, wx.ID_ANY, style=wx.BORDER_NONE)
        self.output_text.SetMinSize(wx.Size(-1, 400))

        # Set wrapping mode
        self.output_text.SetWrapMode(stc.STC_WRAP_CHAR)
        self.output_text.SetUseHorizontalScrollBar(False)
        self.output_text.SetWrapStartIndent(0)
        self.output_text.SetWrapVisualFlags(stc.STC_WRAPVISUALFLAG_END)

        # terminal like style
        self.output_text.StyleSetBackground(stc.STC_STYLE_DEFAULT, wx.BLACK)
        self.output_text.StyleSetForeground(stc.STC_STYLE_DEFAULT, wx.WHITE)
        self.output_text.SetCaretForeground(wx.WHITE)

        # monospaced font
        font = wx.Font(10, wx.FONTFAMILY_TELETYPE, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.output_text.StyleSetFont(stc.STC_STYLE_DEFAULT, font)
        self.output_text.StyleClearAll()

        # disable unused borders/margins
        self.output_text.SetMarginWidth(0, 0)
        self.output_text.SetMarginWidth(1, 0)
        self.output_text.SetMarginWidth(2, 0)

        # set read-only
        self.output_text.SetReadOnly(True)

        bSizer21.Add(self.output_text, wx.SizerFlags().Expand().Border(wx.ALL, 5).Border(wx.RIGHT, 10))

        # define the text communication queue
        self.text_queue = queue.Queue()

        self.upload_button = wx.Button(self.m_panel5, wx.ID_ANY, _('Transfer to PLC'), wx.DefaultPosition, wx.DefaultSize, 0)
        self.upload_button.SetMinSize(wx.Size(150,30))
        self.upload_button.Bind(wx.EVT_BUTTON, self.OnUpload)

        bSizer21.Add(self.upload_button, 0, wx.ALIGN_CENTER|wx.ALL, 5)

        self.m_panel5.SetSizer(bSizer21)
        self.m_panel5.Layout()
        bSizer21.Fit(self.m_panel5)
        self.m_listbook2.AddPage(self.m_panel5, _('Transfer'), True)
        m_listbook2Bitmap = wx.Bitmap(os.path.join(current_dir, "..", "images", "transfer_plc.png"), wx.BITMAP_TYPE_ANY)
        if (m_listbook2Bitmap.IsOk()):
            m_listbook2Images.Add(m_listbook2Bitmap)
            self.m_listbook2.SetPageImage(m_listbook2Index, m_listbook2Index)
            m_listbook2Index += 1

        self.m_panel6 = wx.Panel( self.m_listbook2, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
        bSizer3 = wx.BoxSizer( wx.VERTICAL )

        self.m_staticText4 = wx.StaticText( self.m_panel6, wx.ID_ANY, _('This setting will allow you to change the default pin mapping for your board. Please be cautious while edditing, as mistakes can lead to compilation errors. Pin numbers should obey the Arduino notation for your board and must be comma-separated.'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText4.Wrap( 530 )
        self.m_staticText4.SetMinSize( wx.Size( -1,60 ) )

        bSizer3.Add( self.m_staticText4, 0, wx.ALL, 5 )

        self.m_staticText5 = wx.StaticText( self.m_panel6, wx.ID_ANY, _('Digital Inputs'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText5.Wrap( -1 )
        bSizer3.Add( self.m_staticText5, 0, wx.ALL, 5 )

        self.din_txt = wx.TextCtrl( self.m_panel6, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer3.Add( self.din_txt, 0, wx.ALL|wx.EXPAND, 5 )

        self.m_staticText6 = wx.StaticText( self.m_panel6, wx.ID_ANY, _('Digital Outputs'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText6.Wrap( -1 )
        bSizer3.Add( self.m_staticText6, 0, wx.ALL, 5 )

        self.dout_txt = wx.TextCtrl( self.m_panel6, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer3.Add( self.dout_txt, 0, wx.ALL|wx.EXPAND, 5 )

        self.m_staticText7 = wx.StaticText( self.m_panel6, wx.ID_ANY, _('Analog Inputs'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText7.Wrap( -1 )
        bSizer3.Add( self.m_staticText7, 0, wx.ALL, 5 )

        self.ain_txt = wx.TextCtrl( self.m_panel6, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer3.Add( self.ain_txt, 0, wx.ALL|wx.EXPAND, 5 )

        self.m_staticText8 = wx.StaticText( self.m_panel6, wx.ID_ANY, _('Analog Outputs'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText8.Wrap( -1 )
        bSizer3.Add( self.m_staticText8, 0, wx.ALL, 5 )

        self.aout_txt = wx.TextCtrl( self.m_panel6, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer3.Add( self.aout_txt, 0, wx.ALL|wx.EXPAND, 5 )

        self.m_staticText9 = wx.StaticText( self.m_panel6, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText9.Wrap( -1 )
        self.m_staticText9.SetMinSize( wx.Size( -1,40 ) )

        bSizer3.Add( self.m_staticText9, 0, wx.ALL, 5 )

        gSizer1 = wx.GridSizer( 0, 2, 0, 0 )

        self.m_button2 = wx.Button( self.m_panel6, wx.ID_ANY, _('Restore Defaults'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_button2.SetMinSize( wx.Size( 150,30 ) )
        self.m_button2.Bind(wx.EVT_BUTTON, self.restoreIODefaults)

        gSizer1.Add( self.m_button2, 0, wx.ALIGN_CENTER|wx.ALL, 5 )

        self.m_button3 = wx.Button( self.m_panel6, wx.ID_ANY, _('Save Changes'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_button3.SetMinSize( wx.Size( 150,30 ) )
        self.m_button3.Bind(wx.EVT_BUTTON, self.saveSettings)

        gSizer1.Add( self.m_button3, 0, wx.ALIGN_CENTER|wx.ALL, 5 )

        bSizer3.Add( gSizer1, 1, wx.EXPAND, 5 )

        self.m_panel6.SetSizer( bSizer3 )
        self.m_panel6.Layout()
        bSizer3.Fit( self.m_panel6 )
        self.m_listbook2.AddPage( self.m_panel6, _('I/O Config'), False )
        m_listbook2Bitmap = wx.Bitmap(os.path.join(current_dir, "..", "images", "io.png"), wx.BITMAP_TYPE_ANY )
        if ( m_listbook2Bitmap.IsOk() ):
            m_listbook2Images.Add( m_listbook2Bitmap )
            self.m_listbook2.SetPageImage( m_listbook2Index, m_listbook2Index )
            m_listbook2Index += 1

        self.m_panel7 = wx.Panel( self.m_listbook2, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.TAB_TRAVERSAL )
        bSizer4 = wx.BoxSizer( wx.VERTICAL )

        self.check_modbus_serial = wx.CheckBox( self.m_panel7, wx.ID_ANY, _('Enable Modbus RTU (Serial)'), wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer4.Add( self.check_modbus_serial, 0, wx.ALL, 10 )
        self.check_modbus_serial.Bind(wx.EVT_CHECKBOX, self.onUIChange)

        fgSizer2 = wx.FlexGridSizer( 0, 4, 0, 0 )
        fgSizer2.SetFlexibleDirection( wx.BOTH )
        fgSizer2.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )

        self.m_staticText10 = wx.StaticText( self.m_panel7, wx.ID_ANY, _('Interface:'), wx.DefaultPosition, wx.Size( -1,-1 ), 0 )
        self.m_staticText10.Wrap( -1 )
        self.m_staticText10.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer2.Add( self.m_staticText10, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        serial_iface_comboChoices = [ u"Serial", u"Serial1", u"Serial2", u"Serial3" ]
        self.serial_iface_combo = wx.ComboBox( self.m_panel7, wx.ID_ANY, u"Serial", wx.DefaultPosition, wx.DefaultSize, serial_iface_comboChoices, wx.CB_READONLY )
        self.serial_iface_combo.SetSelection( 0 )
        self.serial_iface_combo.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer2.Add( self.serial_iface_combo, 0, wx.ALL, 5 )

        self.m_staticText11 = wx.StaticText( self.m_panel7, wx.ID_ANY, _('Baudrate:'), wx.DefaultPosition, wx.Size( -1,-1 ), 0 )
        self.m_staticText11.Wrap( -1 )
        self.m_staticText11.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer2.Add( self.m_staticText11, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        baud_rate_comboChoices = [ u"9600", u"14400", u"19200", u"38400", u"57600", u"115200" ]
        self.baud_rate_combo = wx.ComboBox( self.m_panel7, wx.ID_ANY, u"115200", wx.DefaultPosition, wx.DefaultSize, baud_rate_comboChoices, wx.CB_READONLY )
        self.baud_rate_combo.SetSelection( 5 )
        self.baud_rate_combo.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer2.Add( self.baud_rate_combo, 0, wx.ALL, 5 )

        self.m_staticText12 = wx.StaticText( self.m_panel7, wx.ID_ANY, _('Slave ID:'), wx.DefaultPosition, wx.Size( -1,-1 ), 0 )
        self.m_staticText12.Wrap( -1 )
        self.m_staticText12.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer2.Add( self.m_staticText12, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        self.slaveid_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, u"0", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.slaveid_txt.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer2.Add( self.slaveid_txt, 0, wx.ALL, 5 )

        self.m_staticText13 = wx.StaticText( self.m_panel7, wx.ID_ANY, _('RS485 TX Pin:'), wx.DefaultPosition, wx.Size( -1,-1 ), 0 )
        self.m_staticText13.Wrap( -1 )
        self.m_staticText13.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer2.Add( self.m_staticText13, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        self.txpin_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, u"-1", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.txpin_txt.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer2.Add( self.txpin_txt, 0, wx.ALL, 5 )

        self.m_staticText23 = wx.StaticText( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText23.Wrap( -1 )
        self.m_staticText23.SetMaxSize( wx.Size( -1,15 ) )

        fgSizer2.Add( self.m_staticText23, 0, 0, 5 )


        bSizer4.Add( fgSizer2, 0, wx.EXPAND, 5 )

        self.m_staticline21 = wx.StaticLine( self.m_panel7, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
        bSizer4.Add( self.m_staticline21, 0, wx.ALL|wx.BOTTOM|wx.EXPAND|wx.TOP, 5 )

        self.check_modbus_tcp = wx.CheckBox( self.m_panel7, wx.ID_ANY, _('Enable Modbus TCP'), wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer4.Add( self.check_modbus_tcp, 0, wx.ALL, 10 )
        self.check_modbus_tcp.Bind(wx.EVT_CHECKBOX, self.onUIChange)

        fgSizer3 = wx.FlexGridSizer( 0, 2, 0, 0 )
        fgSizer3.SetFlexibleDirection( wx.BOTH )
        fgSizer3.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )

        self.m_staticText14 = wx.StaticText( self.m_panel7, wx.ID_ANY, _('Interface:'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText14.Wrap( -1 )
        self.m_staticText14.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer3.Add( self.m_staticText14, 0, wx.ALL, 5 )

        tcp_iface_comboChoices = [ u"Ethernet", u"WiFi" ]
        self.tcp_iface_combo = wx.ComboBox( self.m_panel7, wx.ID_ANY, u"Ethernet", wx.DefaultPosition, wx.DefaultSize, tcp_iface_comboChoices, wx.CB_READONLY )
        self.tcp_iface_combo.SetSelection( 0 )
        self.tcp_iface_combo.SetMinSize( wx.Size( 560,-1 ) )
        self.tcp_iface_combo.Bind(wx.EVT_COMBOBOX, self.onUIChange)

        fgSizer3.Add( self.tcp_iface_combo, 0, wx.ALL|wx.EXPAND, 5 )

        self.m_staticText15 = wx.StaticText( self.m_panel7, wx.ID_ANY, u"MAC:", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText15.Wrap( -1 )
        self.m_staticText15.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer3.Add( self.m_staticText15, 0, wx.ALL, 5 )

        self.mac_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, u"0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.mac_txt.SetMinSize( wx.Size( 560,-1 ) )

        fgSizer3.Add( self.mac_txt, 0, wx.ALL|wx.EXPAND, 5 )


        bSizer4.Add( fgSizer3, 0, wx.EXPAND, 5 )

        fgSizer4 = wx.FlexGridSizer( 0, 4, 0, 0 )
        fgSizer4.SetFlexibleDirection( wx.BOTH )
        fgSizer4.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )

        self.m_staticText17 = wx.StaticText( self.m_panel7, wx.ID_ANY, u"IP:", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText17.Wrap( -1 )
        self.m_staticText17.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.m_staticText17, 0, wx.ALL, 5 )

        self.ip_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        self.ip_txt.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.ip_txt, 0, wx.ALL, 5 )

        self.m_staticText18 = wx.StaticText( self.m_panel7, wx.ID_ANY, u"DNS:", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText18.Wrap( -1 )
        self.m_staticText18.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.m_staticText18, 0, wx.ALL, 5 )

        self.dns_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        self.dns_txt.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.dns_txt, 0, wx.ALL, 5 )

        self.m_staticText19 = wx.StaticText( self.m_panel7, wx.ID_ANY, _('Gateway:'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText19.Wrap( -1 )
        self.m_staticText19.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.m_staticText19, 0, wx.ALL, 5 )

        self.gateway_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        self.gateway_txt.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.gateway_txt, 0, wx.ALL, 5 )

        self.m_staticText20 = wx.StaticText( self.m_panel7, wx.ID_ANY, _('Subnet:'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText20.Wrap( -1 )
        self.m_staticText20.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.m_staticText20, 0, wx.ALL, 5 )

        self.subnet_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, u"255.255.255.0", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.subnet_txt.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.subnet_txt, 0, wx.ALL, 5 )

        self.m_staticText21 = wx.StaticText( self.m_panel7, wx.ID_ANY, u"WiFi SSID:", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText21.Wrap( -1 )
        self.m_staticText21.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.m_staticText21, 0, wx.ALL, 5 )

        self.wifi_ssid_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        self.wifi_ssid_txt.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.wifi_ssid_txt, 0, wx.ALL, 5 )

        self.m_staticText22 = wx.StaticText( self.m_panel7, wx.ID_ANY, _('WiFi PSK (Password):'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText22.Wrap( -1 )
        self.m_staticText22.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.m_staticText22, 0, wx.ALL, 5 )

        self.wifi_pwd_txt = wx.TextCtrl( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_PASSWORD )
        self.wifi_pwd_txt.SetMinSize( wx.Size( 180,-1 ) )

        fgSizer4.Add( self.wifi_pwd_txt, 0, wx.ALL, 5 )

        self.m_staticText24 = wx.StaticText( self.m_panel7, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText24.Wrap( -1 )
        fgSizer4.Add( self.m_staticText24, 0, wx.ALL, 5 )


        bSizer4.Add( fgSizer4, 1, wx.EXPAND, 5 )

        gSizer2 = wx.GridSizer( 0, 2, 0, 0 )

        self.m_button4 = wx.Button( self.m_panel7, wx.ID_ANY, _('Restore Defaults'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_button4.SetMinSize( wx.Size( 150,30 ) )
        self.m_button4.Bind(wx.EVT_BUTTON, self.restoreCommDefaults)

        gSizer2.Add( self.m_button4, 0, wx.ALIGN_CENTER|wx.ALL, 5 )

        self.m_button5 = wx.Button( self.m_panel7, wx.ID_ANY, _('Save Changes'), wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_button5.SetMinSize( wx.Size( 150,30 ) )
        self.m_button5.Bind(wx.EVT_BUTTON, self.saveSettings)

        gSizer2.Add( self.m_button5, 0, wx.ALIGN_CENTER|wx.ALL, 5 )


        bSizer4.Add( gSizer2, 1, wx.EXPAND, 5 )


        self.m_panel7.SetSizer( bSizer4 )
        self.m_panel7.Layout()
        bSizer4.Fit( self.m_panel7 )
        self.m_listbook2.AddPage( self.m_panel7, _('Communications'), False )
        m_listbook2Bitmap = wx.Bitmap( os.path.join(current_dir, "..", "images", "comm.png"), wx.BITMAP_TYPE_ANY )
        if ( m_listbook2Bitmap.IsOk() ):
            m_listbook2Images.Add( m_listbook2Bitmap )
            self.m_listbook2.SetPageImage( m_listbook2Index, m_listbook2Index )
            m_listbook2Index += 1


        main_sizer.Add( self.m_listbook2, 1, wx.EXPAND |wx.ALL, 0 )


        self.SetSizer(main_sizer)
        main_sizer.Fit(self)
        self.Layout()

        self.Centre( wx.BOTH )

        self.reloadComboChoices(None) # Initialize the com port combo box content, accesses indirectly several elements, which need to be created completely

        self.loadSettings()

        self.set_build_option(self.active_build_option)

    def __del__( self ):
        pass

    def reloadComboChoices(self, event):
        self.setUIState(False)
        current_display = self.com_port_combo.GetValue()
        current_port = next((port for display_text, port in self.com_port_combo_choices.items()
                             if display_text == current_display), current_display)

        self.com_port_combo.Clear()
        self.com_port_combo_choices = {}
        new_display = current_display

        for comport in serial.tools.list_ports.comports():
            display_text = f"{comport.description} ({comport.device})"
            self.com_port_combo_choices[display_text] = comport.device
            if comport.device == current_port:
                new_display = display_text

        self.com_port_combo.SetItems(list(self.com_port_combo_choices.keys()))

        for display_text, port in self.com_port_combo_choices.items():
            if port == current_port:
                new_display = display_text
                break

        wx.CallAfter(self.com_port_combo.SetValue, new_display)

        self.setUIState(True)

    def onBoardChange(self, e):
        self.settings.pop('user_din', None);
        self.settings.pop('user_ain', None);
        self.settings.pop('user_dout', None);
        self.settings.pop('user_aout', None);
        self.onUIChange(e)

    def onBuildCacheOptionChange(self, event):
        """
        Event handler for build cache option changes in the ComboBox.
        Updates the active build option based on user selection.
        """
        selected = self.build_options.GetSelection()
        self.active_build_option = self.BUILD_OPTIONS[selected][1]

    def onUIChange(self, e):
        # Update Comms
        if (self.check_modbus_serial.GetValue() == False):
            self.serial_iface_combo.Enable(False)
            self.baud_rate_combo.Enable(False)
            self.slaveid_txt.Enable(False)
            self.txpin_txt.Enable(False)
        else:
            self.serial_iface_combo.Enable(True)
            self.baud_rate_combo.Enable(True)
            self.slaveid_txt.Enable(True)
            self.txpin_txt.Enable(True)

        if (self.check_compile.GetValue() == False):
            self.com_port_combo.Enable(True)
            self.reload_button.Enable(True)
            self.upload_button.SetLabel(_('Transfer to PLC'))
        else:
            self.com_port_combo.Enable(False)
            self.reload_button.Enable(False)
            self.upload_button.SetLabel(_('Compile'))

        if (self.check_modbus_tcp.GetValue() == False):
            self.tcp_iface_combo.Enable(False)
            self.mac_txt.Enable(False)
            self.ip_txt.Enable(False)
            self.dns_txt.Enable(False)
            self.gateway_txt.Enable(False)
            self.subnet_txt.Enable(False)
            self.wifi_ssid_txt.Enable(False)
            self.wifi_pwd_txt.Enable(False)
        else:
            self.tcp_iface_combo.Enable(True)
            self.mac_txt.Enable(True)
            self.ip_txt.Enable(True)
            self.dns_txt.Enable(True)
            self.gateway_txt.Enable(True)
            self.subnet_txt.Enable(True)
            if (self.tcp_iface_combo.GetValue() == u"Ethernet"):
                self.wifi_ssid_txt.Enable(False)
                self.wifi_pwd_txt.Enable(False)
            elif (self.tcp_iface_combo.GetValue() == u"WiFi"):
                self.wifi_ssid_txt.Enable(True)
                self.wifi_pwd_txt.Enable(True)

        #Update IOs
        board_type = self.board_type_combo.GetValue().split(" [")[0] #remove the trailing [version] on board name
        if board_type not in self.hals:
            board_type = next(iter(self.hals))
        board_din = self.settings.get('user_din', self.hals[board_type]["default_din"])
        board_ain = self.settings.get('user_ain', self.hals[board_type]["default_ain"])
        board_dout = self.settings.get('user_dout', self.hals[board_type]["default_dout"])
        board_aout = self.settings.get('user_aout', self.hals[board_type]["default_aout"])

        self.din_txt.SetValue(str(board_din))
        self.ain_txt.SetValue(str(board_ain))
        self.dout_txt.SetValue(str(board_dout))
        self.aout_txt.SetValue(str(board_aout))

    def send_output_text(self, output):
        self.text_queue.put(output) # queue the text output seperately and thread-safe, as CallAfter() does not preserve the call order
        wx.CallAfter(self.append_text)

    def append_text(self):
        wx.YieldIfNeeded()

        # Append the new text
        self.output_text.SetReadOnly(False)
        while not self.text_queue.empty():
            try:
                line = self.text_queue.get_nowait()
                if line is None:
                    # end of data
                    break
                # we are in the GUI thread context, so we are allowed to append the text directly to `output_text`
                self.output_text.AppendText(line)
                self.text_queue.task_done()
            except queue.Empty:
                break
        self.output_text.SetReadOnly(True)

        self.output_text.ScrollToEnd()
        wx.CallLater(10, self.output_text.ScrollToEnd)

    def restoreIODefaults(self, event):
        board_type = self.board_type_combo.GetValue().split(" [")[0] #remove the trailing [version] on board name
        #print(f'Restoring IO defaults for "{board_type}"')
        self.settings['user_din'] = self.hals[board_type]['default_din']
        self.settings['user_ain'] = self.hals[board_type]['default_ain']
        self.settings['user_dout'] = self.hals[board_type]['default_dout']
        self.settings['user_aout'] = self.hals[board_type]['default_aout']
        self.onUIChange(None)
        self.project_controller.SetArduinoSettingsChanged()

    def set_build_option(self, saved_option: builder.BuildCacheOption):
        """
        Sets the build option in the ComboBox based on saved settings.

        Args:
            saved_option (BuildCacheOption): The build option to be set

        Updates both the ComboBox selection and the active build option.
        """
        self.active_build_option = saved_option
        for index, (_ignored, enum_value) in enumerate(self.BUILD_OPTIONS):
            if enum_value == saved_option:
                self.build_options.SetSelection(index)
                break

    def get_current_build_option(self) -> builder.BuildCacheOption:
        return self.BUILD_OPTIONS[self.build_options.GetSelection()][1]

    def startBuilder(self):
        # Get platform and source_file from hals
        board_type = self.board_type_combo.GetValue().split(" [")[0] #remove the trailing [version] on board name
        board_hal = self.hals[board_type]

        old_values = {
                'last_update': board_hal.get('last_update', None),  # get() if key does not exist
                'version': board_hal.get('version', None)
            }
        
        self.generateDefinitions()

        port = "None" #invalid port
        if (self.check_compile.GetValue() == True):
            port = None
        else:
            selected_port = self.com_port_combo.GetValue()
            # Check and select the port
            port_found = False
            for display_text, port_value in self.com_port_combo_choices.items():
                if selected_port == display_text:
                    port = port_value
                    port_found = True
                    break
            if not port_found:
                port = selected_port  # Use the user entered value directly

        # create a closure to encapsulate self, later this sends the text on behalf of the thread
        def send_text(output):
            self.send_output_text(output)

        # empty the text output for the new build run
        def outputTextClearAll():
            self.output_text.SetReadOnly(False)
            self.output_text.ClearAll()
            self.output_text.SetReadOnly(True)

        wx.CallAfter(outputTextClearAll)
        wx.YieldIfNeeded()

        # now create the build thread
        compiler_thread = threading.Thread(
            target=builder.build,
            args=(self.plc_program, self.definitions, self.arduino_sketch, port, send_text, board_hal, 
                  self.active_build_option)
        )
        compiler_thread.start()
        compiler_thread.join()
        
        values_changed = (
            old_values['last_update'] != board_hal.get('last_update', None) or 
            old_values['version'] != board_hal.get('version', None)
        )
        
        if values_changed:
            self.saveHals()

        self.saveSettings()
        self.updateInstalledBoards()
        self.loadSettings() # Get the correct board name if an update or install occurred

        # reset the build cache option and enable the UI
        wx.CallAfter(self.set_build_option, builder.BuildCacheOption.USE_CACHE)
        wx.CallAfter(wx.CallLater, 100, self.setUIState, True)

    def setUIState(self, enabled):
        self.board_type_combo.Enable(enabled)
        self.check_compile.Enable(enabled)
        if (not enabled or self.check_compile.GetValue() == False):
            self.com_port_combo.Enable(enabled)
        self.reload_button.Enable(enabled)
        self.upload_button.Enable(enabled)
        self.build_options.Enable(enabled)

    def OnUpload(self, event):
        self.setUIState(False)
        builder_thread = threading.Thread(target=self.startBuilder)
        builder_thread.start()

    # def generateDefinitionsFile(self):
    #
    #     if platform.system() == 'Windows':
    #         base_path = 'editor\\arduino\\examples\\Baremetal\\'
    #     else:
    #         base_path = 'editor/arduino/examples/Baremetal/'
    #
    #     #Store program MD5 on target
    #     define_file = '//Program MD5\n'
    #     define_file += '#define PROGRAM_MD5 "' + str(self.md5) + '"\n'
    #
    #     #Generate Communication Config defines
    #     define_file += '//Comms configurations\n'
    #
    #     define_file += '#define MBSERIAL_IFACE ' + str(self.serial_iface_combo.GetValue()) + '\n'
    #     define_file += '#define MBSERIAL_BAUD ' + str(self.baud_rate_combo.GetValue()) + '\n'
    #     define_file += '#define MBSERIAL_SLAVE ' + str(self.slaveid_txt.GetValue()) + '\n'
    #     define_file += '#define MBTCP_MAC ' + str(self.mac_txt.GetValue()) + '\n'
    #     define_file += '#define MBTCP_IP ' + str(self.ip_txt.GetValue()).replace('.',',') + '\n'
    #     define_file += '#define MBTCP_DNS ' + str(self.dns_txt.GetValue()).replace('.',',') + '\n'
    #     define_file += '#define MBTCP_GATEWAY ' + str(self.gateway_txt.GetValue()).replace('.',',') + '\n'
    #     define_file += '#define MBTCP_SUBNET ' + str(self.subnet_txt.GetValue()).replace('.',',') + '\n'
    #     define_file += '#define MBTCP_SSID "' + str(self.wifi_ssid_txt.GetValue()) + '"\n'
    #     define_file += '#define MBTCP_PWD "' + str(self.wifi_pwd_txt.GetValue()) + '"\n'
    #
    #     if (self.check_modbus_serial.GetValue() == True):
    #         define_file += '#define MBSERIAL\n'
    #         define_file += '#define MODBUS_ENABLED\n'
    #
    #     if (self.txpin_txt.GetValue() != '-1'):
    #         define_file += '#define MBSERIAL_TXPIN ' + str(self.txpin_txt.GetValue()) + '\n'
    #
    #     if (self.check_modbus_tcp.GetValue() == True):
    #         define_file += '#define MBTCP\n'
    #         define_file += '#define MODBUS_ENABLED\n'
    #         if (self.tcp_iface_combo.GetValue() == u"Ethernet"):
    #             define_file += '#define MBTCP_ETHERNET\n'
    #         elif (self.tcp_iface_combo.GetValue() == u'WiFi'):
    #             define_file += '#define MBTCP_WIFI\n'
    #
    #     #Generate IO Config defines
    #     define_file += '\n\n//IO Config\n'
    #     define_file += '#define PINMASK_DIN ' + str(self.din_txt.GetValue()) + '\n'
    #     define_file += '#define PINMASK_AIN ' + str(self.ain_txt.GetValue()) + '\n'
    #     define_file += '#define PINMASK_DOUT ' + str(self.dout_txt.GetValue()) + '\n'
    #     define_file += '#define PINMASK_AOUT ' + str(self.aout_txt.GetValue()) + '\n'
    #     define_file += '#define NUM_DISCRETE_INPUT ' + str(len(str(self.din_txt.GetValue()).split(','))) + '\n'
    #     define_file += '#define NUM_ANALOG_INPUT ' + str(len(str(self.ain_txt.GetValue()).split(','))) + '\n'
    #     define_file += '#define NUM_DISCRETE_OUTPUT ' + str(len(str(self.dout_txt.GetValue()).split(','))) + '\n'
    #     define_file += '#define NUM_ANALOG_OUTPUT ' + str(len(str(self.aout_txt.GetValue()).split(','))) + '\n'
    #
    #     # Get define from hals
    #     board_type = self.board_type_combo.GetValue().split(" [")[0]
    #     if 'define' in self.hals[board_type]:
    #         define_file += '#define '+ self.hals[board_type]['define'] +'\n'
    #
    #     define_file += '\n\n//Arduino Libraries\n'
    #
    #     #Generate Arduino Libraries defines
    #     if (self.plc_program.find('DS18B20;') > 0) or (self.plc_program.find('DS18B20_2_OUT;') > 0) or (self.plc_program.find('DS18B20_3_OUT;') > 0) or (self.plc_program.find('DS18B20_4_OUT;') > 0) or (self.plc_program.find('DS18B20_5_OUT;') > 0):
    #         define_file += '#define USE_DS18B20_BLOCK\n'
    #     if (self.plc_program.find('P1AM_INIT;') > 0):
    #         define_file += '#define USE_P1AM_BLOCKS\n'
    #     if (self.plc_program.find('CLOUD_BEGIN;') > 0):
    #         define_file += '#define USE_CLOUD_BLOCKS\n'
    #     if (self.plc_program.find('MQTT_CONNECT;') > 0) or (self.plc_program.find('MQTT_CONNECT_AUTH;') > 0):
    #         define_file += '#define USE_MQTT_BLOCKS\n'
    #     if (self.plc_program.find('ARDUINOCAN_CONF;') > 0):
    #         define_file += '#define USE_ARDUINOCAN_BLOCK\n'
    #     elif (self.plc_program.find('ARDUINOCAN_WRITE;') > 0):
    #         define_file += '#define USE_ARDUINOCAN_BLOCK\n'
    #     elif (self.plc_program.find('ARDUINOCAN_WRITE_WORD;') > 0):
    #         define_file += '#define USE_ARDUINOCAN_BLOCK\n'
    #     elif (self.plc_program.find('ARDUINOCAN_READ;') > 0):
    #         define_file += '#define USE_ARDUINOCAN_BLOCK\n'
    #     if (self.plc_program.find('STM32CAN_CONF;') > 0):
    #         define_file += '#define USE_STM32CAN_BLOCK\n'
    #     elif (self.plc_program.find('STM32CAN_WRITE;') > 0):
    #         define_file += '#define USE_STM32CAN_BLOCK\n'
    #     elif (self.plc_program.find('STM32CAN_READ;') > 0):
    #         define_file += '#define USE_STM32CAN_BLOCK\n'
    #
    #     #Generate Arduino Extension (sketch) define
    #     if self.arduino_sketch != None:
    #         define_file += '#define USE_ARDUINO_SKETCH\n'
    #         define_file += '#define ARDUINO_PLATFORM\n'
    #         #Copy the sketch contents to the .h file
    #         f = open(os.path.join(base_path, 'ext', 'arduino_sketch.h'), 'w')
    #         f.write(self.arduino_sketch)
    #
    #     #Write file to disk
    #     f = open(base_path+'defines.h', 'w')
    #     f.write(define_file)
    #     f.flush()
    #     f.close()

    def generateDefinitions(self):
        """Generate definitions and store them in the object"""
        self.definitions = []  # Reset definitions array
        
        # Program MD5
        self.definitions.extend([
            '//Program MD5',
            f'#define PROGRAM_MD5 "{str(self.md5)}"'
        ])

        # Communication Config defines
        self.definitions.extend([
            '//Comms configurations',
            f'#define MBSERIAL_IFACE {str(self.serial_iface_combo.GetValue())}',
            f'#define MBSERIAL_BAUD {str(self.baud_rate_combo.GetValue())}',
            f'#define MBSERIAL_SLAVE {str(self.slaveid_txt.GetValue())}',
            f'#define MBTCP_MAC {str(self.mac_txt.GetValue())}',
            f'#define MBTCP_IP {str(self.ip_txt.GetValue()).replace(".",",")}',
            f'#define MBTCP_DNS {str(self.dns_txt.GetValue()).replace(".",",")}',
            f'#define MBTCP_GATEWAY {str(self.gateway_txt.GetValue()).replace(".",",")}',
            f'#define MBTCP_SUBNET {str(self.subnet_txt.GetValue()).replace(".",",")}',
            f'#define MBTCP_SSID "{str(self.wifi_ssid_txt.GetValue())}"',
            f'#define MBTCP_PWD "{str(self.wifi_pwd_txt.GetValue())}"'
        ])

        if self.check_modbus_serial.GetValue():
            self.definitions.extend([
                '#define MBSERIAL',
                '#define MODBUS_ENABLED'
            ])

        if self.txpin_txt.GetValue() != '-1':
            self.definitions.append(f'#define MBSERIAL_TXPIN {str(self.txpin_txt.GetValue())}')

        if self.check_modbus_tcp.GetValue():
            self.definitions.append('#define MBTCP')
            self.definitions.append('#define MODBUS_ENABLED')
            if self.tcp_iface_combo.GetValue() == "Ethernet":
                self.definitions.append('#define MBTCP_ETHERNET')
            elif self.tcp_iface_combo.GetValue() == 'WiFi':
                self.definitions.append('#define MBTCP_WIFI')

        # IO Config defines
        self.definitions.extend([
            '\n\n//IO Config',
            f'#define PINMASK_DIN {str(self.din_txt.GetValue())}',
            f'#define PINMASK_AIN {str(self.ain_txt.GetValue())}',
            f'#define PINMASK_DOUT {str(self.dout_txt.GetValue())}',
            f'#define PINMASK_AOUT {str(self.aout_txt.GetValue())}',
            f'#define NUM_DISCRETE_INPUT {str(len(str(self.din_txt.GetValue()).split(",")))}',
            f'#define NUM_ANALOG_INPUT {str(len(str(self.ain_txt.GetValue()).split(",")))}',
            f'#define NUM_DISCRETE_OUTPUT {str(len(str(self.dout_txt.GetValue()).split(",")))}',
            f'#define NUM_ANALOG_OUTPUT {str(len(str(self.aout_txt.GetValue()).split(",")))}'
        ])

        # Get define from hals
        board_type = self.board_type_combo.GetValue().split(" [")[0]
        if 'define' in self.hals[board_type]:
            board_define = self.hals[board_type]['define']
            # Handle both string and array cases
            if isinstance(board_define, str):
                self.definitions.append(f'#define {board_define}')
            elif isinstance(board_define, list):
                self.definitions.extend([f'#define {define}' for define in board_define])

        # Arduino Libraries defines
        self.definitions.append('\n\n//Arduino Libraries')

        # Check for required libraries in PLC program
        if any(lib in self.plc_program for lib in ['DS18B20;', 'DS18B20_2_OUT;', 'DS18B20_3_OUT;', 'DS18B20_4_OUT;', 'DS18B20_5_OUT;']):
            self.definitions.append('#define USE_DS18B20_BLOCK')
        if 'P1AM_INIT;' in self.plc_program:
            self.definitions.append('#define USE_P1AM_BLOCKS')
        if 'CLOUD_BEGIN;' in self.plc_program:
            self.definitions.append('#define USE_CLOUD_BLOCKS')
        if 'MQTT_CONNECT;' in self.plc_program or 'MQTT_CONNECT_AUTH;' in self.plc_program:
            self.definitions.append('#define USE_MQTT_BLOCKS')
        if any(can_func in self.plc_program for can_func in ['ARDUINOCAN_CONF;', 'ARDUINOCAN_WRITE;', 'ARDUINOCAN_WRITE_WORD;', 'ARDUINOCAN_READ;']):
            self.definitions.append('#define USE_ARDUINOCAN_BLOCK')
        if any(stm_func in self.plc_program for stm_func in ['STM32CAN_CONF;', 'STM32CAN_WRITE;', 'STM32CAN_READ;']):
            self.definitions.append('#define USE_STM32CAN_BLOCK')


    def saveSettings(self, event=None):
        self.settings['board_type'] = self.board_type_combo.GetValue().split(" [")[0] #remove the trailing [version] on board name
        self.settings['user_din'] = str(self.din_txt.GetValue())
        self.settings['user_ain'] = str(self.ain_txt.GetValue())
        self.settings['user_dout'] = str(self.dout_txt.GetValue())
        self.settings['user_aout'] = str(self.aout_txt.GetValue())

        com_port_value = self.com_port_combo.GetValue()
        self.settings['com_port'] = next((port for display_text, port in self.com_port_combo_choices.items()
                                     if display_text == com_port_value), com_port_value)

        self.settings['mb_serial'] = self.check_modbus_serial.GetValue()
        self.settings['serial_iface'] = self.serial_iface_combo.GetValue()
        self.settings['baud'] = self.baud_rate_combo.GetValue()
        self.settings['slaveid'] = self.slaveid_txt.GetValue()
        self.settings['txpin'] = self.txpin_txt.GetValue()
        self.settings['mb_tcp'] = self.check_modbus_tcp.GetValue()
        self.settings['tcp_iface'] = self.tcp_iface_combo.GetValue()
        self.settings['mac'] = self.mac_txt.GetValue()
        self.settings['ip'] = self.ip_txt.GetValue()
        self.settings['dns'] = self.dns_txt.GetValue()
        self.settings['gateway'] = self.gateway_txt.GetValue()
        self.settings['subnet'] = self.subnet_txt.GetValue()
        self.settings['ssid'] = self.wifi_ssid_txt.GetValue()
        self.settings['pwd'] = self.wifi_pwd_txt.GetValue()

        # Remove last_update from settings since it is now managed in hals.json
        self.settings.pop('last_update', None)

        self.project_controller.SetArduinoSettingsChanged()

    def loadSettings(self):
        self.settings = self.project_controller.GetArduinoSettings()

        if not self.settings:
            self.restoreIODefaults(None)
            self.restoreCommDefaults(None)

        # # Check if should update subsystem
        # if ('last_update' in self.settings.keys()):
        #     self.last_update = self.settings['last_update']
        #     if (time.time() - float(self.last_update) > 604800.0): #604800 is the number of seconds in a week (7 days)
        #         self.update_subsystem = True
        #         self.last_update = time.time()
        #     else:
        #         self.update_subsystem = False
        # else:
        #     self.update_subsystem = True
        #     self.last_update = time.time()

        # Get the correct name for the board_type
        board = self.settings['board_type'].split(' [')[0]
        board_name = ""
        if board in self.hals:
            if self.hals[board]['version'] == "0":
                board_name = board + ' [' + _('NOT INSTALLED') + ']'
            else:
                board_name = board + ' [' + self.hals[board]['version'] + ']'

        wx.CallAfter(self.board_type_combo.SetValue, board_name)

        com_port_value = self.settings['com_port']
        for display_text, port in self.com_port_combo_choices.items():
            if port == com_port_value:
                com_port_value = display_text
                break
        wx.CallAfter(self.com_port_combo.SetValue, com_port_value)

        wx.CallAfter(self.check_modbus_serial.SetValue, self.settings['mb_serial'])
        wx.CallAfter(self.serial_iface_combo.SetValue, self.settings['serial_iface'])
        wx.CallAfter(self.baud_rate_combo.SetValue, self.settings['baud'])
        wx.CallAfter(self.slaveid_txt.SetValue, self.settings['slaveid'])
        wx.CallAfter(self.txpin_txt.SetValue, self.settings['txpin'])
        wx.CallAfter(self.check_modbus_tcp.SetValue, self.settings['mb_tcp'])
        wx.CallAfter(self.tcp_iface_combo.SetValue, self.settings['tcp_iface'])
        wx.CallAfter(self.mac_txt.SetValue, self.settings['mac'])
        wx.CallAfter(self.ip_txt.SetValue, self.settings['ip'])
        wx.CallAfter(self.dns_txt.SetValue, self.settings['dns'])
        wx.CallAfter(self.gateway_txt.SetValue, self.settings['gateway'])
        wx.CallAfter(self.subnet_txt.SetValue, self.settings['subnet'])
        wx.CallAfter(self.wifi_ssid_txt.SetValue, self.settings['ssid'])
        wx.CallAfter(self.wifi_pwd_txt.SetValue, self.settings['pwd'])

        wx.CallAfter(self.onUIChange, None)

    def restoreCommDefaults(self, event):
        # Read default settings from settingsDefaults.json
        if platform.system() == 'Windows':
            base_path = 'editor\\arduino\\examples\\Baremetal\\'
        else:
            base_path = 'editor/arduino/examples/Baremetal/'

        default_settings_file = os.path.join(base_path, 'settingsDefaults.json')
        settings_file = os.path.join(base_path, 'settings.json')

        if os.path.exists(default_settings_file):
            with open(default_settings_file, 'r') as f:
                default_settings = json.load(f)

            # Update only the communication-related settings
            # Preserve non-communication settings
            for key in self.settings:
                if key not in ['mb_serial', 'serial_iface', 'baud', 'slaveid', 'txpin',
                               'mb_tcp', 'tcp_iface', 'mac', 'ip', 'dns', 'gateway',
                               'subnet', 'ssid', 'pwd']:
                    default_settings[key] = self.settings[key]

            # Write the updated settings back to the settings
            for key in default_settings:
                self.settings[key] = default_settings[key]

            # inform project controller about the changes
            self.project_controller.SetArduinoSettingsChanged()

            # Use loadSettings to update the GUI
            self.loadSettings()

            # Enable all communication-related fields
            self.serial_iface_combo.Enable(True)
            self.baud_rate_combo.Enable(True)
            self.slaveid_txt.Enable(True)
            self.txpin_txt.Enable(True)
            self.tcp_iface_combo.Enable(True)
            self.mac_txt.Enable(True)
            self.ip_txt.Enable(True)
            self.dns_txt.Enable(True)
            self.gateway_txt.Enable(True)
            self.subnet_txt.Enable(True)
            self.wifi_ssid_txt.Enable(True)
            self.wifi_pwd_txt.Enable(True)

            # Call onUIChange to update the state of the fields based on checkbox values
            self.onUIChange(None)
        else:
            print("Default settings file not found:", default_settings_file)

    def loadHals(self):
        """Load HALs list from json file"""
        if platform.system() == 'Windows':
            jfile = 'editor\\arduino\\examples\\Baremetal\\hals.json'
        else:
            jfile = 'editor/arduino/examples/Baremetal/hals.json'
    
        with open(jfile, 'r') as f:
            self.hals = json.load(f)


    def saveHals(self):
        """Save HALs list to json file"""
        if platform.system() == 'Windows':
            jfile = 'editor\\arduino\\examples\\Baremetal\\hals.json'
        else:
            jfile = 'editor/arduino/examples/Baremetal/hals.json'
        
        with open(jfile, 'w') as f:
            json.dump(self.hals, f, indent=2, sort_keys=True)
            f.flush()

    def updateInstalledBoards(self):
        """Update the list of installed boards using arduino-cli's JSON output"""
        if platform.system() == 'Windows':
            cli_command = 'editor\\arduino\\bin\\arduino-cli-w64.exe'
        elif platform.system() == 'Darwin':
            cli_command = 'editor/arduino/bin/arduino-cli-mac'
        else:
            cli_command = 'editor/arduino/bin/arduino-cli-l64'
    
        # Get core list in JSON format
        core_list = builder.runCommand(cli_command + ' --json core list')
    
        if core_list == None or core_list == '':
            print("Error reading core list")
            return
    
        # Parse JSON output
        core_data = json.loads(core_list)

        # Check if platforms key exists and is an array
        if 'platforms' not in core_data or not isinstance(core_data['platforms'], list):
            # No installed platforms or invalid format - mark all as not installed
            for board in self.hals:
                self.hals[board]['version'] = '0'
            self.saveHals()
            return

        # Update installed boards in list
        for board in self.hals:
            board_core = self.hals[board]['core']
            version = '0'  # Default to not installed

            # Look for matching platform in JSON data
            for arduino_platform in core_data['platforms']:
                if arduino_platform['id'] == board_core:
                    version = arduino_platform['installed_version']
                    break

            self.hals[board]['version'] = version

        self.saveHals()

    # # Check if should update subsystem
    # if ('last_update' in board_hal):
    #     last_update = board_hal['last_update']
    #     if (time.time() - float(last_update) > 604800.0): #604800 is the number of seconds in a week (7 days)
    #         update_subsystem = True
    #     else:
    #         update_subsystem = False
    #
    # board_hal['last_update'] = time.time()
    # board_hal['version'] = get_core_version(core)


