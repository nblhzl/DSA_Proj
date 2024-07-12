# postal_code_utils.py

import pandas as pd

# Load postal code data
postal_code_data = pd.read_csv('SG_Postal.csv')

def get_coordinates(postal_code):
    entry = postal_code_data[postal_code_data['postal_code'] == postal_code]
    if entry.empty:
        return None
    return entry.iloc[0]['lat'], entry.iloc[0]['lon']
