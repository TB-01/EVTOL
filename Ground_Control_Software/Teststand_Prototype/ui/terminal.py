# ui/terminal.py
# A tiny terminal: entry with history. Calls on_command(text) when Enter is pressed.

import customtkinter as ctk

class TerminalFrame(ctk.CTkFrame):
    def __init__(self, master, on_command):
        super().__init__(master)
        self.on_command = on_command
        self.entry = ctk.CTkEntry(self, placeholder_text="Type commands (e.g., /help, /ping, /snapshot, /connect COM6 921600, /hex 45 01 ...)")
        self.entry.pack(side="left", fill="x", expand=True, padx=(0,8), pady=6)
        self.send_btn = ctk.CTkButton(self, text="Send", width=80, command=self._send)
        self.send_btn.pack(side="left", pady=6)

        self.entry.bind("<Return>", lambda e: self._send())
        self.entry.bind("<Up>", self._history_prev)
        self.entry.bind("<Down>", self._history_next)

        self._hist = []
        self._hist_i = 0

    def focus_input(self):
        self.entry.focus_set()

    def _send(self):
        txt = self.entry.get().strip()
        if not txt:
            return
        # history
        if not self._hist or self._hist[-1] != txt:
            self._hist.append(txt)
        self._hist_i = len(self._hist)
        # dispatch
        try:
            self.on_command(txt)
        finally:
            self.entry.delete(0, "end")

    def _history_prev(self, _evt=None):
        if not self._hist: return
        self._hist_i = max(0, self._hist_i - 1)
        self.entry.delete(0,"end")
        self.entry.insert(0, self._hist[self._hist_i])

    def _history_next(self, _evt=None):
        if not self._hist: return
        self._hist_i = min(len(self._hist), self._hist_i + 1)
        self.entry.delete(0,"end")
        if self._hist_i < len(self._hist):
            self.entry.insert(0, self._hist[self._hist_i])
