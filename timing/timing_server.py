#!/usr/bin/python
import argparse
import os
import sys

def log_to_csv(log_clean, log_csv):
    """
    Reads from clean (no color code) log and print CSV content to outfile.
    Assumptions: 
        File is opened elsewhere.
        Log is synchronized (timing for 1 image is in consecutive lines)
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
    outfile.write("overall,SURF,closest_match,PnP,PnP_global,PnP_update_total,PnP_update_1,PnP_update_2\n")

    # write timing data
    i = 0
    while i < len(lines):
        KEYWORDS = ["overall", "surf", "closest_match", "pnp", "pnpglobal", "pnpupdates", "pnpupdate1", "pnpupdate2"] # keywords in log file

        results_good = True
        for j in range(len(KEYWORDS)):
            if lines[i+j][-2].lower() != KEYWORDS[j].lower():
                results_good = False
                break

        if results_good:
            overall = int(lines[i][-1])
            surf = int(lines[i+1][-1])
            closest = int(lines[i+2][-1])
            pnp = int(lines[i+3][-1])
            pnp_global = int(lines[i+4][-1])
            pnp_updates = int(lines[i+5][-1])
            pnp_update_1 = int(lines[i+6][-1])
            pnp_update_2 = int(lines[i+7][-1])
            outfile.write("{},{},{},{},{},{},{},{}\n".format(overall, surf, closest, pnp, pnp_global, pnp_updates, pnp_update_1, pnp_update_2))

        i += 8

    # close file except for stdout
    if len(log_csv):
        outfile.close()

def sanitize_log(tag_timing, log_input, log_clean):
    #log_sanitized, log_tmp):
    """
    Convert raw log file (with color codes) to clean log file.
    """
    CMD_RM_CR = "sed 's/\r$//'"
    CMD_ASCII_ONLY = "sed -r 's/\x1B\[([0-9]{1,2}(;[0-9]{1,2})?)?[m|K]//g'"
    
    # clean log to remove carriage return and non-ascii characters (color codes)
    os.system('cat {} | grep {} | {} | {} > {}'.format(log_input, tag_timing,
                CMD_RM_CR, CMD_ASCII_ONLY, log_clean))

def _main(args):
    parser = argparse.ArgumentParser(description="Extract timing data from server log.")
    parser.add_argument("log_file", type=str, help="Server log file")
    parser.add_argument("-o", "--output", type=str, default="",
                        help="Filename of CSV output. Outputs to STDOUT if not given.")
    args = parser.parse_args()

    LOG_TMP = "log_tmp.tmp"
    TAG_TIMING = "TAG_TIME"

    sanitize_log(TAG_TIMING, args.log_file, LOG_TMP)
    log_to_csv(LOG_TMP, args.output)

    # delete tmp file
    os.unlink(LOG_TMP)

if __name__ == '__main__':
    _main(sys.argv[1:])
