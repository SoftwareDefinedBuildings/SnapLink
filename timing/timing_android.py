#!/usr/bin/python
import argparse
import os
import sys

def log_to_csv(log_clean, log_csv):
    """
    Reads from filtered log and print CSV content to outfile.
    """
    lines = None
    try:
        with open(log_clean, "r") as infile:
            lines = infile.read().split("\n")
            lines = [line.split(" ") for line in lines if len(line)] # remove empty lines
    except:
        print("ERROR: Could not open log file")
        sys.exit(1)

    outfile = None
    try:
        if len(log_csv) > 0:
            outfile = open(log_csv, "w")
        else:
            outfile = sys.stdout
    except:
        print("Could not open CSV file for write")
        sys.exit(1)

    # write CSV header
    outfile.write("overall\n")

    # write timing data
    i = 0
    while i < len(lines):
        KEYWORDS = ["capture", "response"] # keywords in log file

        results_good = True
        for j in range(len(KEYWORDS)):
            # keyword in log has colon at end
            if lines[i+j][-2].lower() != KEYWORDS[j].lower():
                results_good = False
                break

        if results_good:
            capture = int(lines[i][-1])
            response = int(lines[i+1][-1])
            outfile.write("{}\n".format(response-capture))

        i += 2

    # close file except for stdout
    if len(log_csv):
        outfile.close()

def filter_log(tag_timing, log_input, log_clean):
    """
    Filter out logcat output to only keep timing lines
    """
    CMD_RM_CR = "sed 's/\r$//'" # remove carriage returns
    os.system('cat {} | grep {} | {} > {}'.format(log_input, tag_timing,
                                                    CMD_RM_CR, log_clean))

def _main(args):
    parser = argparse.ArgumentParser(description="Extract timing data from android log.")
    parser.add_argument("log_file", type=str, help="Android log file")
    parser.add_argument("-o", "--output", type=str, default="",
                        help="Filename of CSV output. Outputs to STDOUT if not given.")
    args = parser.parse_args()

    LOG_TMP = "log_tmp.tmp"
    TAG_TIMING = "TAG_TIME"

    filter_log(TAG_TIMING, args.log_file, LOG_TMP)
    log_to_csv(LOG_TMP, args.output)

    # delete tmp file
    os.unlink(LOG_TMP)

if __name__ == '__main__':
    _main(sys.argv[1:])
