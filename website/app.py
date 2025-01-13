import sqlite3
import json
from flask import Flask, redirect, render_template, Response, request, jsonify, stream_with_context
import time

app = Flask(__name__)

# this function is for receiveing the data from the master
@app.route('/api/data', methods=['POST'])
def receive_data():
    print("Received headers:", request.headers)
    print("Received data:", request.get_data())
    if not request.is_json: # make sure it's json
        return jsonify({"error": "Request must be JSON"}), 400
        
    data = request.get_json()
    print("data from arduino:", data, flush=True)
    # had to make them short because it takes too much time to transmit full names like "slave_num" then just a number etc
    slave_num = data.get('sn')
    speed = data.get('s')
    lrotation = data.get('lr')
    rrotation = data.get('rr')
    action = data.get('a')
        
    if not all([slave_num, speed, lrotation, rrotation, action]): # if not all data are here return an error
        return jsonify({"error": "Missing required fields"}), 400
    
    try:
        insert_into_db(slave_num, speed, lrotation, rrotation, action) # just try put them into the DB
    except sqlite3.Error as e:
        print(f"Database insert failed: {e}")
    
    return jsonify({"message": "Data received successfully"}), 200

# this is the function that is important for the 
@app.route('/stream')
def stream():
    def generate():
        while True:
            # always get the latest data from the DB
            conn = get_db_conn()
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM robot ORDER BY created_at DESC LIMIT 1") # since I use timestamps it's easiest with it
            data = cursor.fetchone()
            conn.close()

            if data:
                json_data = json.dumps({ # here because of the row_factory i can just call data[column] instead of data[0], data[1], etc very cool!
                    'slave_num': data['slave_num'],
                    'speed': data['speed'],
                    'lrotation': data['lrotation'],
                    'rrotation': data['rrotation'],
                    'action': data['action'],
                    'time': data['created_at']
                })
                yield f"data: {json_data}\n\n"
            else:
                print(data,"Data could not be retrieved", flush=True)
            time.sleep(0.5)  # Wait before next update
            yield ": heartbeat\n\n"

    return Response(stream_with_context(generate()), mimetype="text/event-stream")

def get_db_conn():
    conn = sqlite3.connect("robot.db")
    conn.row_factory = sqlite3.Row # I just learned that this is very cool because i can call the colunm names with this
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
                created_at DATETIME DEFAULT CURRENT_TIMESTAMP)''')
    
    db.commit()
    db.close()

def insert_into_db(slave_num, speed, lrotation, rrotation, action):
    conn = get_db_conn()
    cursor = conn.cursor()
    try:
        cursor.execute('''INSERT INTO robot (slave_num, speed, lrotation, rrotation, action) 
                          VALUES (?, ?, ?, ?, ?)''', (slave_num, speed, lrotation, rrotation, action))
        conn.commit()
    except sqlite3.Error as e:
        print(f"Error inserting into database: {e}",flush=True) # for debugging and testing
    finally:
        conn.close()

init_db()

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
    app.run(debug=True)
