"""wx_splash.py — the apps' start-up splash: a borderless, always-on-top
window showing one image, with its corners cut round where the platform
supports window shapes (X11, Windows; Wayland ignores SetShape and shows
the plain rectangle).

    splash = show_splash(path_to_png)
    ...                        # build the main frame
    splash.Destroy()

Returns a real window even if the image is missing (a small blank one), so
the caller's Destroy() never has to be guarded.
"""

import wx

RADIUS = 16


def _rounded_region(width, height, radius):
    """A wx.Region of the rounded rectangle, built from a mask bitmap."""
    bmp = wx.Bitmap(width, height)
    dc = wx.MemoryDC(bmp)
    dc.SetBackground(wx.Brush(wx.BLACK))
    dc.Clear()
    dc.SetBrush(wx.Brush(wx.WHITE))
    dc.SetPen(wx.Pen(wx.WHITE))
    dc.DrawRoundedRectangle(0, 0, width, height, radius)
    dc.SelectObject(wx.NullBitmap)
    return wx.Region(bmp, wx.BLACK)


class Splash(wx.Frame):

    def __init__(self, bitmap, radius=RADIUS):
        style = (wx.FRAME_SHAPED | wx.BORDER_NONE | wx.STAY_ON_TOP
                 | wx.FRAME_NO_TASKBAR)
        super().__init__(None, wx.ID_ANY, '', style=style)
        self.bitmap = bitmap
        self.SetClientSize(bitmap.GetSize())
        self.Bind(wx.EVT_PAINT, self._on_paint)
        if radius:
            region = _rounded_region(bitmap.GetWidth(), bitmap.GetHeight(),
                                     radius)
            # Before the window is realised on GTK the shape is lost; set it
            # now and again once the window exists.
            self.SetShape(region)
            self.Bind(wx.EVT_WINDOW_CREATE,
                      lambda evt: self.SetShape(region))
        self.CentreOnScreen()

    def _on_paint(self, event):
        wx.PaintDC(self).DrawBitmap(self.bitmap, 0, 0, True)


def show_splash(path, radius=RADIUS):
    """Show the splash for the image at *path*; returns the window."""
    image = wx.Image(path, wx.BITMAP_TYPE_ANY) if path else wx.Image()
    if not image.IsOk():
        image = wx.Image(320, 180)
    splash = Splash(wx.Bitmap(image), radius)
    splash.Show()
    splash.Update()
    return splash
