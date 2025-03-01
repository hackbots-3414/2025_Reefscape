import csv
from os import sep
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

class VisionLog:
    def __init__(self):
        self.logs = []
        self.sources = set()

    def add_log(self, log):
        if len(log) != 5:
            print("found bad log entry")
            return

        timestamp = float(log[0])
        source = log[1].strip()
        x = float(log[2])
        y = float(log[3])
        err = float(log[4])

        self.logs.append(LogEntry()
            .withTimestamp(timestamp)
            .withSource(source)
            .withRobotPose(x, y)
            .withError(err)
        )

        self.sources.add(source)

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

    def split_sources(self):
        logs = []
        for source in self.sources:
            log = VisionLog()
            log.logs = self.get_source(source)
            log.sources.add(source)
            logs.append(log) 
        return logs

logs = VisionLog.from_csv("vision.log")
separated = logs.split_sources()
for log in separated:
    source = log.sources
    timestamp = []
    err = []
    for entry in log.logs:
        err.append(entry.err)
        timestamp.append(entry.timestamp)
    plt.plot(timestamp, err, label=str(source))

# add a recommendation
plt.axhline(0.2, label="acceptable range", color="gray")

plt.xlabel("FPGA time")
plt.ylabel("Error")
plt.legend()
plt.show()

