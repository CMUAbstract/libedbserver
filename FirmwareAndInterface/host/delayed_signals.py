import signal

class DelayedSignals(object):

    def __init__(self, signals=[signal.SIGINT]):
        self.signals = signals

    def __enter__(self):
        self.signal_received = False
        self.old_handlers = {}
        for sig in self.signals:
            self.old_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, self.handler)

    def handler(self, signal, frame):
        self.signal_received = (signal, frame)

    def __exit__(self, type, value, traceback):
        for sig in self.signals:
            signal.signal(sig, self.old_handlers[sig])
        if self.signal_received:
            self.old_handlers[self.signal_received[0]](*self.signal_received)
