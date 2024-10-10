import csv
import os


class CSVReader:
    def __init__(self, csv_file_path):
        self.csv_file_path = csv_file_path

        # Check if the file exists and is readable
        if not os.path.exists(self.csv_file_path):
            raise FileNotFoundError(f"File not found: {self.csv_file_path}")

        # Initialize file and reader
        self.open_file()

    def open_file(self):
        """Opens the file and initializes the CSV reader and header."""
        self.file = open(self.csv_file_path, mode="r")
        self.csv_reader = csv.reader(self.file)

        try:
            # Read and store the header (first row)
            self.header = next(self.csv_reader)
        except StopIteration:
            raise ValueError(f"CSV file '{self.csv_file_path}' is empty.")

    def reset_iterator(self):
        """Closes the current file and reopens it to reset the iterator."""
        self.file.close()
        self.open_file()

    def get_next_row(self):
        """Iteraties to next row of CSV, looping back to start if end-of-file i sreached."""
        try:
            return next(self.csv_reader)
        except StopIteration:
            self.reset_iterator()
            return self.get_next_row()  # Get the first row after resetting

    def close(self):
        """Closes the file when done."""
        self.file.close()
