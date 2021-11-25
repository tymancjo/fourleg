
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>
        Mousebot
    </title>
    <meta name="viewport" content="user-scalable=no">
</head>
<body  style="position: fixed; font-family: 'Gill Sans', 'Gill Sans MT', Calibri, 'Trebuchet MS', sans-serif ;
color:rgb(128, 128, 128);
font-size: xx-large;">
<center>
    <h1 style="text-align:center">
        Aron Jog </h1>
    <p style="text-align: center;">
        FB: <span id="speed"> </span> 
        LR: <span id="angle"> </span>
    </p>
    <p>Move modes:</p>

      <div>
        <input type="radio" id="m1" name="mode" value="w"
               checked>
        <label for="m1">Walk</label>

        <input type="radio" id="m2" name="mode" value="circ">
        <label for="m2">Swing</label>
      
        <input type="radio" id="m3" name="mode" value="up">
        <label for="m3">Bounce</label>
      
        <input type="radio" id="m4" name="mode" value="twist">
        <label for="m4">Twist</label>
      </div>

      <canvas id="canvas" name="game"></canvas>
 </center>    
<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  window.addEventListener('load', onLoad);
  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage; // <-- add this line
  }
  function onOpen(event) {
    console.log('Connection opened');
  }
  function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
  }
  function onMessage(event) {
    var state;
    if (event.data == "1"){
      state = "ON";
    }
    else{
      state = "OFF";
    }
    document.getElementById('state').innerHTML = state;
  }
  function onLoad(event) {
    initWebSocket();
    initButton();
  }
  function initButton() {
    document.getElementById('button').addEventListener('click', toggle);
  }
  function toggle(){
    websocket.send('toggle');
  }

  function send(x,y){

            var command = "w";
            var the_mode = document.querySelector('input[name="mode"]:checked'); 
            if(the_mode) {
              command = the_mode.value;
              }

            var data = "w "+x+" "+y+"\n";
            
            switch (command) {
                        case 'w':
                          data = "w "+x+" "+y+"\n";
                          break;
                        
                        case 'circ':
                          x = -Math.round( 40*x/100 );
                          y =  Math.round( 50*y/100 );
                          data = "circ "+x+" "+y+"\n";
                          break;
                          
                        case 'up':
                          var up = Math.round( 70*x/100 );
                          var fr = Math.round( 70*y/100 );

                          x = up + fr;
                          y = up - fr;

                          
                          data = "up "+x+" "+y+"\n";
                          break;  
      
                        case 'twist':
                          x = Math.round( 35*x/100 );
                          y = Math.round( 35*y/100 );
                          data = "twist "+y+"\n";
                          break;
                        
                        default:
                          console.log(`Sorry, we are out of ${expr}.`);
                      } 
              
            
            
//            data = JSON.stringify(data);
            console.log(data);
            websocket.send(data);
  }
  
</script>

<script>
        var canvas, ctx;

        window.addEventListener('load', () => {

            canvas = document.getElementById('canvas');
            ctx = canvas.getContext('2d');          
            resize(); 

            document.addEventListener('mousedown', startDrawing);
            document.addEventListener('mouseup', stopDrawing);
            document.addEventListener('mousemove', Draw);

            document.addEventListener('touchstart', startDrawing);
            document.addEventListener('touchend', stopDrawing);
            document.addEventListener('touchcancel', stopDrawing);
            document.addEventListener('touchmove', Draw);
            window.addEventListener('resize', resize);

            document.getElementById("x_coordinate").innerText = 0;
            document.getElementById("y_coordinate").innerText = 0;
            document.getElementById("speed").innerText = 0;
            document.getElementById("angle").innerText = 0;
        });

      


        var width, height, radius, x_orig, y_orig;
        function resize() {
            width = window.innerWidth / 2;
            radius = 100;
            height = radius * 6.5;
            ctx.canvas.width = width;
            ctx.canvas.height = height;
            background();
            joystick(width / 2, height / 3);
        }

        function background() {
            x_orig = width / 2;
            y_orig = height / 3;

            ctx.beginPath();
            ctx.arc(x_orig, y_orig, radius + 20, 0, Math.PI * 2, true);
            ctx.fillStyle = '#ECE5E5';
            ctx.fill();
        }

        function joystick(width, height) {
            ctx.beginPath();
            ctx.arc(width, height, radius, 0, Math.PI * 2, true);
            ctx.fillStyle = '#F08080';
            ctx.fill();
            ctx.strokeStyle = '#F6ABAB';
            ctx.lineWidth = 8;
            ctx.stroke();
        }

        let coord = { x: 0, y: 0 };
        let paint = false;

        function getPosition(event) {
            var mouse_x = event.clientX || event.touches[0].clientX;
            var mouse_y = event.clientY || event.touches[0].clientY;
            coord.x = mouse_x - canvas.offsetLeft;
            coord.y = mouse_y - canvas.offsetTop;
        }

        function is_it_in_the_circle() {
            var current_radius = Math.sqrt(Math.pow(coord.x - x_orig, 2) + Math.pow(coord.y - y_orig, 2));
            if (radius >= current_radius) return true
            else return false
        }


        function startDrawing(event) {
            paint = true;
            getPosition(event);
            if (is_it_in_the_circle()) {
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                background();
                joystick(coord.x, coord.y);
                Draw();
            }
        }


        function stopDrawing() {
            paint = false;
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            background();
            joystick(width / 2, height / 3);
            document.getElementById("speed").innerText = 0;
            document.getElementById("angle").innerText = 0;
            send(0,0);

        }

        function Draw(event) {

            if (paint) {
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                background();
                var angle_in_degrees,x, y, speed;
                var angle = Math.atan2((coord.y - y_orig), (coord.x - x_orig));

                if (Math.sign(angle) == -1) {
                    angle_in_degrees = Math.round(-angle * 180 / Math.PI);
                }
                else {
                    angle_in_degrees =Math.round( 360 - angle * 180 / Math.PI);
                }


                if (is_it_in_the_circle()) {
                    joystick(coord.x, coord.y);
                    x = coord.x;
                    y = coord.y;
                }
                else {
                    x = radius * Math.cos(angle) + x_orig;
                    y = radius * Math.sin(angle) + y_orig;
                    joystick(x, y);
//                    x = 2 * x;
//                    y = 2 * y
                }
              
//                    joystick(coord.x, coord.y);
//                    x = Math.min(100, Math.max(-100, x - x_orig));
//                    y = Math.min(100, Math.max(-100, y - y_orig));
                    x = x - x_orig;
                    y = y - y_orig;
                
                                
                getPosition(event);

                var lr = Math.round(100 * (x) / radius);
                var fb = Math.round(100 * (y) / radius);
                    lr = lr > 100 ? 100 : lr < -100 ? -100 : lr;
                    fb = fb > 100 ? 100 : fb < -100 ? -100 : fb;

                

                document.getElementById("speed").innerText = fb;
                document.getElementById("angle").innerText = lr;

                
                send(-fb,lr);
            }
        } 
    </script>
</body>
</html>

</body>
</html>
)rawliteral";
