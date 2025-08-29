import csv
import sys
from matplotlib import pyplot as plt

class LogEntry:
    def withTimestamp(self, timestamp):
        self.timestamp = timestamp
        return self

    def withSource(self, source):
        self.source = source
        return self

    def withRobotPose(self, x, y):
        self.x = x
        self.y = y
        return self
    
    def withError(self, err):
        self.err = err
        return self

    def withMode(self, mode):
        self.mode = mode
        return self

class VisionLog:
    def __init__(self):
        self.logs = []
        self.sources = set()
        self.modes = set()
        self.source = None
        self.mode = None

    def add_log(self, log):
        if len(log) != 6:
            print("found bad log entry")
            return

        timestamp = float(log[0])
        source = log[1]
        x = float(log[2])
        y = float(log[3])
        err = float(log[4])
        mode = log[5]

        self.logs.append(LogEntry()
            .withTimestamp(timestamp)
            .withSource(source)
            .withRobotPose(x, y)
            .withError(err)
            .withMode(mode)
        )

        self.sources.add(source)
        self.modes.add(mode)

    @classmethod
    def from_csv(cls, filepath):
        logs = cls()

        with open(filepath) as logfile:
            rdr = csv.reader(logfile)
            # ignore the header row
            _ = next(rdr, None)
            for line in rdr:
                logs.add_log(line)

        return logs

    def get_source(self, source):
        ret = []
        for log in self.logs:
            if log.source == source:
                ret.append(log)
        return ret

    def get_mode(self, mode):
        ret = []
        for log in self.logs:
            if log.mode == mode:
                ret.append(log)
        return ret

    def split_sources(self):
        logs = []
        for source in self.sources:
            log = VisionLog()
            log.logs = self.get_source(source)
            log.sources.add(source)
            log.source = source
            logs.append(log) 
        return logs

    def split_modes(self):
        logs = []
        for mode in self.modes:
            log = VisionLog()
            log.logs = self.get_mode(mode)
            log.modes.add(mode)
            log.mode = mode
            logs.append(log)
        return logs


if len(sys.argv) != 2:
    print(f"usage: {sys.argv[0]} <filename>")
    exit()

filepath = sys.argv[1]

logs = VisionLog.from_csv(filepath)
source_separated = logs.split_sources()
mode_separated = logs.split_modes()

def graph_error():
    # add a recommendation
    plt.axhline(0.2, label="acceptable range", color="gray")
    plt.title("Vision Error")
    plt.xlabel("FPGA time")
    plt.ylabel("Error")
    plt.legend()
    plt.show()

def graph_error_by_source():
    for log in source_separated:
        source = log.source
        timestamp = []
        err = []
        for entry in log.logs:
            err.append(entry.err)
            timestamp.append(entry.timestamp)
        plt.scatter(timestamp, err, label=source, alpha=0.5)

    graph_error()


def graph_error_by_mode():
    for log in mode_separated:
        mode = log.mode
        timestamp = []
        err = []
        for entry in log.logs:
            err.append(entry.err)
            timestamp.append(entry.timestamp)
        plt.scatter(timestamp, err, label=mode, alpha=0.5)
    graph_error()

def graph_counts():
    for log in source_separated:
        source = log.source
        size = len(log.logs)
        plt.bar(source, size)

    plt.show()

def graph_positions():
    x = [entry.x for entry in logs.logs]
    y = [entry.y for entry in logs.logs]
    plt.scatter(x, y)
    plt.plot(x, y)

    plt.show()

graph_counts()
graph_error_by_source()
graph_error_by_mode()
graph_positions()
