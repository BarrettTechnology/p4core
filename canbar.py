"""
canbar.py — the CAN port bar the puck GUIs share (p4gui and pucktuner).

    CAN Port: [can0, USB (CANable), up v]  [Scan Pucks]  Select ID: [4 v]

CanBar builds the widgets on a parent window and lays them out in
``.sizer``; the app puts that sizer where it wants the bar and adds its own
controls after it (p4gui's Sync Rate and CAN FD).  The bar knows the port
list (can_backend.port_choices) and the found node IDs; connecting and
scanning stay with the app, which binds the bar's events to its own
handlers (CanBar.bind).

GUI only: this module imports wx, which the rest of p4core does not need.
"""

import wx

from . import can_backend


class CanBar:
    """CAN Port, Scan Pucks and Select ID on *parent*.

    The widgets are attributes (label_can_port, choice_port, btn_scan,
    label_select_id, choice_id) so an app can enable, lock or test them
    directly.  Select ID shows 'None' until a scan finds a puck."""

    NONE = 'None'

    def __init__(self, parent, ports=None):
        self.label_can_port = wx.StaticText(parent, label='CAN Port:')
        self.choice_port = wx.Choice(parent)
        self.btn_scan = wx.Button(parent, label='Scan Pucks')
        self.label_select_id = wx.StaticText(parent, label='Select ID:')
        self.choice_id = wx.Choice(parent, choices=[self.NONE])
        self.choice_id.SetSelection(0)

        self.sizer = wx.BoxSizer(wx.HORIZONTAL)
        for widget, gap in ((self.label_can_port, 2), (self.choice_port, 8),
                            (self.btn_scan, 8), (self.label_select_id, 2),
                            (self.choice_id, 8)):
            self.sizer.Add(widget, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, gap)
        self.fill_ports(ports)

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------
    def bind(self, on_scan=None, on_select=None, on_port=None):
        """Scan Pucks -> *on_scan*, a Select ID choice -> *on_select*, a CAN
        Port choice -> *on_port*; each gets the wx event."""
        for widget, event, handler in (
                (self.btn_scan, wx.EVT_BUTTON, on_scan),
                (self.choice_id, wx.EVT_CHOICE, on_select),
                (self.choice_port, wx.EVT_CHOICE, on_port)):
            if handler is not None:
                widget.Bind(event, handler)

    # ------------------------------------------------------------------
    # CAN Port
    # ------------------------------------------------------------------
    def fill_ports(self, ports=None, select=None):
        """List *ports* ({'port', 'label'} dicts; port_choices() by default)
        and select the one named *select*, else the first."""
        if ports is None:
            ports = can_backend.port_choices()
        self.choice_port.Clear()
        for info in ports:
            self.choice_port.Append(info['label'], info['port'])
        if not (select and self.select_port(select, add=False)) \
                and self.choice_port.GetCount():
            self.choice_port.SetSelection(0)

    def port(self):
        """The selected port's name (as make_network takes it), or None."""
        sel = self.choice_port.GetSelection()
        if sel == wx.NOT_FOUND:
            return None
        return self.choice_port.GetClientData(sel)

    def select_port(self, port, add=True):
        """Select *port*; one not listed (an adapter not plugged in yet) is
        added under its bare name when *add*.  True if it is selected.
        None selects no port (port() is then None), for an app that must not
        open a port before the user picks one."""
        if port is None:
            self.choice_port.SetSelection(wx.NOT_FOUND)
            return False
        for i in range(self.choice_port.GetCount()):
            if self.choice_port.GetClientData(i) == port:
                self.choice_port.SetSelection(i)
                return True
        if not add:
            return False
        self.choice_port.SetSelection(self.choice_port.Append(port, port))
        return True

    # ------------------------------------------------------------------
    # Select ID
    # ------------------------------------------------------------------
    def set_nodes(self, nodes, select=None):
        """List the found node IDs *nodes* and select *select* if it is among
        them, else the first; 'None' when there are none.  Returns the
        selected ID (or None).  Like any SetSelection this fires no event:
        the caller selects the puck itself."""
        nodes = sorted(int(n) for n in nodes)
        self.choice_id.Clear()
        if not nodes:
            self.choice_id.Append(self.NONE)
            self.choice_id.SetSelection(0)
            return None
        for n in nodes:
            self.choice_id.Append(str(n))
        chosen = select if select in nodes else nodes[0]
        self.choice_id.SetSelection(nodes.index(chosen))
        return chosen

    def selected_id(self):
        """The selected node ID, or None."""
        try:
            return int(self.choice_id.GetStringSelection())
        except ValueError:
            return None

    def nodes(self):
        """The node IDs listed."""
        return [int(s) for s in self.choice_id.GetItems() if s != self.NONE]
