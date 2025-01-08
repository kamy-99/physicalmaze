import sqlite3
import json
from flask import Flask, redirect, render_template, Response, request, jsonify

def create_app():
    app = Flask(__name__)

    init_db()
    
    return app

def get_db_conn():
    conn = sqlite3.connect("robot.db")
    conn.row_factory = sqlite3.Row
    return conn

def init_db():
    db = sqlite3.connect("robot.db")
    cursor = db.cursor()
    cursor.execute('''CREATE TABLE IF NOT EXISTS robot (
                slave_num INTEGER NOT NULL,
                speed INTEGER NOT NULL,
                lrotation INTEGER NOT NULL,
                rrotation INTEGER NOT NULL,
                action TEXT NOT NULL,
                time DATETIME DEFAULT CURRENT_TIMESTAMP)''')
    
    db.commit()
    db.close()

app = create_app()

def insert_into_db(slave_num, speed, lrotation, rrotation, action):
    conn = get_db_conn()
    cursor = conn.cursor()
    try:
        cursor.execute('''INSERT INTO robot (slave_num, speed, lrotation, rrotation, action) 
                          VALUES (?, ?, ?, ?, ?)''', (slave_num, speed, lrotation, rrotation, action))
        conn.commit()
    except Exception as e:
        print(f"Error inserting into database: {e}") # for testing
    finally:
        conn.close()

@app.route('/')
def index():
    return render_template("index.html")

@app.route("/linefollower")
def line():
    return render_template("linefollower.html")

@app.route("/linemaze")
def maze():
    return render_template("linemaze.html")

@app.route("/physicalmaze")
def pmaze():
    return render_template("pmaze.html")

if __name__ == "__main__":
    app.run(debug=True) # for debugging
