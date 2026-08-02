# CDJ-100S style display for SSD1322 256x64 OLED
# CircuitPython — draws directly into a displayio.Bitmap
#
# Usage:
#   from cdj100s_display import CDJDisplay
#   cdj = CDJDisplay(display)
#   cdj.update(track=1, time_str="1:23.45", bpm=128.0, pitch=0.0,
#               progress=0.35, playing=True, remain=False)

import displayio

# ---------------------------------------------------------------------------
# 7-segment lookup: segments = [a, b, c, d, e, f, g]
#   a=top, b=top-right, c=bot-right, d=bottom,
#   e=bot-left, f=top-left, g=middle
# ---------------------------------------------------------------------------
_SEGS = {
    '0': (1,1,1,1,1,1,0),
    '1': (0,1,1,0,0,0,0),
    '2': (1,1,0,1,1,0,1),
    '3': (1,1,1,1,0,0,1),
    '4': (0,1,1,0,0,1,1),
    '5': (1,0,1,1,0,1,1),
    '6': (1,0,1,1,1,1,1),
    '7': (1,1,1,0,0,0,0),
    '8': (1,1,1,1,1,1,1),
    '9': (1,1,1,1,0,1,1),
    '-': (0,0,0,0,0,0,1),
    ' ': (0,0,0,0,0,0,0),
}

# Palette indices
_BG   = 0   # black
_DIM  = 1   # very dim green  (~6% brightness)
_MID  = 2   # mid green       (~40% brightness)
_ON   = 3   # full green      (100% brightness)

# ---------------------------------------------------------------------------
# Tiny 4x6 pixel font for small labels (uppercase + digits + symbols)
# Each character is a list of 6 rows, each row is a 4-bit int (MSB=left)
# ---------------------------------------------------------------------------
_FONT4 = {
    'A': [0b0110,0b1001,0b1111,0b1001,0b1001,0b0000],
    'B': [0b1110,0b1001,0b1110,0b1001,0b1110,0b0000],
    'C': [0b0111,0b1000,0b1000,0b1000,0b0111,0b0000],
    'D': [0b1110,0b1001,0b1001,0b1001,0b1110,0b0000],
    'E': [0b1111,0b1000,0b1110,0b1000,0b1111,0b0000],
    'F': [0b1111,0b1000,0b1110,0b1000,0b1000,0b0000],
    'G': [0b0111,0b1000,0b1011,0b1001,0b0111,0b0000],
    'H': [0b1001,0b1001,0b1111,0b1001,0b1001,0b0000],
    'I': [0b1110,0b0100,0b0100,0b0100,0b1110,0b0000],
    'K': [0b1001,0b1010,0b1100,0b1010,0b1001,0b0000],
    'L': [0b1000,0b1000,0b1000,0b1000,0b1111,0b0000],
    'M': [0b1001,0b1111,0b1111,0b1001,0b1001,0b0000],
    'N': [0b1001,0b1101,0b1011,0b1001,0b1001,0b0000],
    'P': [0b1110,0b1001,0b1110,0b1000,0b1000,0b0000],
    'R': [0b1110,0b1001,0b1110,0b1010,0b1001,0b0000],
    'S': [0b0111,0b1000,0b0110,0b0001,0b1110,0b0000],
    'T': [0b1110,0b0100,0b0100,0b0100,0b0100,0b0000],
    'U': [0b1001,0b1001,0b1001,0b1001,0b0110,0b0000],
    'Y': [0b1001,0b1001,0b0110,0b0100,0b0100,0b0000],
    '0': [0b0110,0b1001,0b1001,0b1001,0b0110,0b0000],
    '1': [0b0010,0b0110,0b0010,0b0010,0b0111,0b0000],
    '2': [0b0110,0b1001,0b0010,0b0100,0b1111,0b0000],
    '3': [0b1110,0b0001,0b0110,0b0001,0b1110,0b0000],
    '4': [0b1001,0b1001,0b1111,0b0001,0b0001,0b0000],
    '5': [0b1111,0b1000,0b1110,0b0001,0b1110,0b0000],
    '6': [0b0111,0b1000,0b1110,0b1001,0b0110,0b0000],
    '7': [0b1111,0b0001,0b0010,0b0100,0b0100,0b0000],
    '8': [0b0110,0b1001,0b0110,0b1001,0b0110,0b0000],
    '9': [0b0110,0b1001,0b0111,0b0001,0b0110,0b0000],
    '.': [0b0000,0b0000,0b0000,0b0000,0b0100,0b0000],
    ':': [0b0000,0b0100,0b0000,0b0100,0b0000,0b0000],
    '+': [0b0000,0b0100,0b1110,0b0100,0b0000,0b0000],
    '-': [0b0000,0b0000,0b1110,0b0000,0b0000,0b0000],
    '%': [0b1001,0b0010,0b0100,0b1001,0b0000,0b0000],
    ' ': [0b0000,0b0000,0b0000,0b0000,0b0000,0b0000],
    '>': [0b1000,0b0100,0b0010,0b0100,0b1000,0b0000],  # play triangle
    '|': [0b1010,0b1010,0b1010,0b1010,0b1010,0b0000],  # pause bars
}


class CDJDisplay:
    """
    Renders a CDJ-100S style UI onto an SSD1322 256x64 display.

    Parameters
    ----------
    display : adafruit_ssd1322.SSD1322
        The initialised display object.
    """

    def __init__(self, display):
        self.display = display

        # 4-colour grayscale palette: BG, DIM, MID, ON
        self._palette = displayio.Palette(4)
        self._palette[_BG]  = 0x000000
        self._palette[_DIM] = 0x0a200a
        self._palette[_MID] = 0x207020
        self._palette[_ON]  = 0x00ff40

        self._bitmap = displayio.Bitmap(256, 64, 4)

        group = displayio.Group()
        group.append(displayio.TileGrid(self._bitmap, pixel_shader=self._palette))
        display.root_group = group

        self._blink = True   # call toggle_blink() from your main loop

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def toggle_blink(self):
        """Call this every ~500 ms to drive the pause blink."""
        self._blink = not self._blink

    def update(self, track=1, time_str="0:00.00", bpm=128.0,
               pitch=0.0, progress=0.0, playing=True, remain=False):
        """
        Redraw the entire display.

        Parameters
        ----------
        track    : int   — track number (1-99)
        time_str : str   — elapsed/remain time, format "M:SS.FF"
        bpm      : float — beats per minute
        pitch    : float — pitch offset in % (-8.0 to +8.0)
        progress : float — playback position 0.0-1.0
        playing  : bool  — True = playing, False = paused (blinks)
        remain   : bool  — True = REMAIN mode lit
        """
        bmp = self._bitmap
        W, H = 256, 64

        # Clear
        for i in range(W * H):
            bmp[i % W, i // W] = _BG

        # Background dim fill for progress bar area
        self._hline(0, 54, W, _DIM)

        # ── Track number (top-left, large 7-seg, 2 digits) ──────────────
        tr = ("  " + str(max(1, min(99, int(track)))))[-2:]
        self._draw_seg(2,  2, tr[0], w=10, h=18)
        self._draw_seg(14, 2, tr[1], w=10, h=18)
        self._draw_small(2, 22, "TRACK", _DIM)

        # ── Status labels ────────────────────────────────────────────────
        self._draw_small(28, 6,  "REMAIN", _ON if remain else _DIM)
        # Play/pause indicator (blinks when paused)
        if playing:
            self._draw_small(28, 13, "> PLAY", _ON)
        else:
            col = _ON if self._blink else _DIM
            self._draw_small(28, 13, "| PAUSE", col)
        self._draw_small(28, 20, "A.CUE", _DIM)

        # ── Time display (large 7-seg, MM:SS.FF) ─────────────────────────
        parts = time_str.split(':')
        mm = ("00" + (parts[0] if parts else "0"))[-2:]
        rest = parts[1] if len(parts) > 1 else "00.00"
        sf = rest.split('.')
        ss = ("00" + sf[0])[-2:]
        ff = ("00" + (sf[1][:2] if len(sf) > 1 else "00"))[-2:]

        tx, ty, tw, th = 78, 1, 13, 22
        cx = tx
        for ch in mm + ':' + ss + '.' + ff:
            if ch in (':', '.'):
                # two-dot separator
                bmp[cx,   ty + th//3]     = _ON
                bmp[cx,   ty + th//3 + 1] = _ON
                bmp[cx,   ty + 2*th//3]   = _ON
                bmp[cx,   ty + 2*th//3+1] = _ON
                cx += 4
            else:
                self._draw_seg(cx, ty, ch, w=tw, h=th)
                cx += tw + 2

        # ── BPM (smaller 7-seg, top right) ───────────────────────────────
        bpm_str = ("   " + str(int(round(float(bpm)))))[-3:]
        bx = 196
        for i, ch in enumerate(bpm_str):
            if ch != ' ':
                self._draw_seg(bx + i * 11, 2, ch, w=9, h=14)
        self._draw_small(196, 20, "BPM", _DIM)

        # ── Pitch bar (far right, vertical) ──────────────────────────────
        px, py, ph = 242, 2, 38
        for y in range(ph):
            bmp[px + 1, py + y] = _DIM
            bmp[px + 2, py + y] = _DIM
            bmp[px + 3, py + y] = _DIM
            bmp[px + 4, py + y] = _DIM

        center_y = py + ph // 2
        pitch_clamped = max(-8.0, min(8.0, pitch))
        fill_px = int(abs(pitch_clamped) / 8.0 * (ph // 2))
        if pitch_clamped >= 0:
            for y in range(center_y - fill_px, center_y + 1):
                for dx in range(1, 5):
                    bmp[px + dx, y] = _ON
        else:
            for y in range(center_y, center_y + fill_px + 1):
                for dx in range(1, 5):
                    bmp[px + dx, y] = _ON

        # Center tick (notch)
        for dx in range(5):
            bmp[px + dx, center_y] = _MID

        # Pitch % label
        sign = '+' if pitch >= 0 else '-'
        self._draw_small(px - 2, py + ph + 4,
                         sign + str(int(abs(pitch))) + '%', _ON)

        # ── Progress bar (bottom) ─────────────────────────────────────────
        prog_px = int(max(0.0, min(1.0, progress)) * W)
        for x in range(W):
            col = _ON if x < prog_px else _DIM
            for y in range(54, 60):
                bmp[x, y] = col

        # Quarter tick marks
        for t in range(1, 4):
            tx2 = W * t // 4
            for y in range(54, 60):
                bmp[tx2, y] = _BG

        # Playhead marker (vertical line + triangle above)
        for y in range(51, 60):
            bmp[prog_px, y] = _ON
        if 0 < prog_px < W - 1:
            bmp[prog_px - 1, 52] = _ON
            bmp[prog_px + 1, 52] = _ON

    # ------------------------------------------------------------------
    # Partial update methods — each redraws only its own region
    # ------------------------------------------------------------------

    def update_progress(self, progress=0.0):
        """Redraw only the bottom progress bar (y=51-59)."""
        bmp = self._bitmap
        W = 256
        prog_px = int(max(0.0, min(1.0, progress)) * W)

        for x in range(W):
            col = _ON if x < prog_px else _DIM
            for y in range(54, 60):
                bmp[x, y] = col

        # Quarter tick marks
        for t in range(1, 4):
            tx = W * t // 4
            for y in range(54, 60):
                bmp[tx, y] = _BG

        # Playhead marker
        for y in range(51, 60):
            bmp[prog_px, y] = _ON
        if 0 < prog_px < W - 1:
            bmp[prog_px - 1, 52] = _ON
            bmp[prog_px + 1, 52] = _ON

    def update_time(self, time_str="0:00.00"):
        """Redraw only the large time digits (x=78-194, y=1-22)."""
        bmp = self._bitmap
        # Clear the time region first
        self._fill_rect(78, 1, 116, 23, _BG)

        parts = time_str.split(':')
        mm = ("00" + (parts[0] if parts else "0"))[-2:]
        rest = parts[1] if len(parts) > 1 else "00.00"
        sf = rest.split('.')
        ss = ("00" + sf[0])[-2:]
        ff = ("00" + (sf[1][:2] if len(sf) > 1 else "00"))[-2:]

        tx, ty, tw, th = 78, 1, 13, 22
        cx = tx
        for ch in mm + ':' + ss + '.' + ff:
            if ch in (':', '.'):
                bmp[cx, ty + th//3]      = _ON
                bmp[cx, ty + th//3 + 1]  = _ON
                bmp[cx, ty + 2*th//3]    = _ON
                bmp[cx, ty + 2*th//3+1]  = _ON
                cx += 4
            else:
                self._draw_seg(cx, ty, ch, w=tw, h=th)
                cx += tw + 2

    def update_bpm(self, bpm=128.0):
        """Redraw only the BPM digits (x=196-233, y=2-20)."""
        self._fill_rect(196, 2, 37, 19, _BG)
        bpm_str = ("   " + str(int(round(float(bpm)))))[-3:]
        for i, ch in enumerate(bpm_str):
            if ch != ' ':
                self._draw_seg(196 + i * 11, 2, ch, w=9, h=14)
        self._draw_small(196, 20, "BPM", _DIM)

    def update_track(self, track=1):
        """Redraw only the track number (x=2-25, y=2-27)."""
        self._fill_rect(2, 2, 23, 26, _BG)
        tr = ("  " + str(max(1, min(99, int(track)))))[-2:]
        self._draw_seg(2,  2, tr[0], w=10, h=18)
        self._draw_seg(14, 2, tr[1], w=10, h=18)
        self._draw_small(2, 22, "TRACK", _DIM)

    def update_pitch(self, pitch=0.0):
        """Redraw only the pitch bar and label (x=240-255, y=2-48)."""
        bmp = self._bitmap
        px, py, ph = 242, 2, 38
        self._fill_rect(px - 2, py, 16, ph + 12, _BG)

        for y in range(ph):
            for dx in range(1, 5):
                bmp[px + dx, py + y] = _DIM

        center_y = py + ph // 2
        pitch_clamped = max(-8.0, min(8.0, pitch))
        fill_px = int(abs(pitch_clamped) / 8.0 * (ph // 2))
        if pitch_clamped >= 0:
            for y in range(center_y - fill_px, center_y + 1):
                for dx in range(1, 5):
                    bmp[px + dx, y] = _ON
        else:
            for y in range(center_y, center_y + fill_px + 1):
                for dx in range(1, 5):
                    bmp[px + dx, y] = _ON

        for dx in range(5):
            bmp[px + dx, center_y] = _MID

        sign = '+' if pitch >= 0 else '-'
        self._draw_small(px - 2, py + ph + 4,
                         sign + str(int(abs(pitch))) + '%', _ON)

    def update_play_state(self, playing=True):
        """Redraw only the play/pause label (x=28-76, y=13-18)."""
        self._fill_rect(28, 13, 49, 6, _BG)
        if playing:
            self._draw_small(28, 13, "> PLAY", _ON)
        else:
            col = _ON if self._blink else _DIM
            self._draw_small(28, 13, "| PAUSE", col)

    def update_remain(self, remain=False):
        """Redraw only the REMAIN label (x=28-76, y=6-11)."""
        self._fill_rect(28, 6, 49, 6, _BG)
        self._draw_small(28, 6, "REMAIN", _ON if remain else _DIM)

    # ------------------------------------------------------------------
    # Drawing primitives
    # ------------------------------------------------------------------

    def _hline(self, x, y, w, color):
        for i in range(w):
            self._bitmap[x + i, y] = color

    def _vline(self, x, y, h, color):
        for i in range(h):
            self._bitmap[x, y + i] = color

    def _fill_rect(self, x, y, w, h, color):
        bmp = self._bitmap
        for dy in range(h):
            for dx in range(w):
                bmp[x + dx, y + dy] = color

    def _draw_seg(self, x, y, ch, w=10, h=18):
        """Draw a single 7-segment character at pixel position (x, y)."""
        segs = _SEGS.get(ch, _SEGS[' '])
        bmp = self._bitmap
        sw = max(1, w // 6)   # segment thickness
        g  = 1                 # gap at corners

        def hrect(rx, ry, rw, rh):
            self._fill_rect(rx, ry, max(1, rw), max(1, rh), _ON)

        def hrect_dim(rx, ry, rw, rh):
            self._fill_rect(rx, ry, max(1, rw), max(1, rh), _DIM)

        half = h // 2

        # Draw all segments dim first
        hrect_dim(x + g + sw,  y,              w - 2*(g+sw), sw)      # a top
        hrect_dim(x + w - sw,  y + g + sw,     sw, half - g - sw)     # b top-right
        hrect_dim(x + w - sw,  y + half + g,   sw, half - g - sw)     # c bot-right
        hrect_dim(x + g + sw,  y + h - sw,     w - 2*(g+sw), sw)      # d bottom
        hrect_dim(x,           y + half + g,   sw, half - g - sw)     # e bot-left
        hrect_dim(x,           y + g + sw,     sw, half - g - sw)     # f top-left
        hrect_dim(x + g + sw,  y + half - sw//2, w - 2*(g+sw), sw)    # g middle

        # Draw lit segments on top
        if segs[0]: hrect(x + g + sw,  y,              w - 2*(g+sw), sw)
        if segs[1]: hrect(x + w - sw,  y + g + sw,     sw, half - g - sw)
        if segs[2]: hrect(x + w - sw,  y + half + g,   sw, half - g - sw)
        if segs[3]: hrect(x + g + sw,  y + h - sw,     w - 2*(g+sw), sw)
        if segs[4]: hrect(x,           y + half + g,   sw, half - g - sw)
        if segs[5]: hrect(x,           y + g + sw,     sw, half - g - sw)
        if segs[6]: hrect(x + g + sw,  y + half - sw//2, w - 2*(g+sw), sw)

    def _draw_small(self, x, y, text, color):
        """Draw small 4×6 pixel text."""
        bmp = self._bitmap
        cx = x
        for ch in text.upper():
            glyph = _FONT4.get(ch, _FONT4[' '])
            for row, bits in enumerate(glyph):
                for col in range(4):
                    if bits & (0b1000 >> col):
                        px_ = cx + col
                        py_ = y + row
                        if 0 <= px_ < 256 and 0 <= py_ < 64:
                            bmp[px_, py_] = color
            cx += 5   # 4px glyph + 1px gap
