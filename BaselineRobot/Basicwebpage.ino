// following is a default webpage if html file load from SD card fails. 
// It's intended to show a video stream on port 81
static const char PROGMEM BASIC[] = R"rawliteral(
<html>
  <head>
    <title>Default Robot Page</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
  </head>
  <body>
    <h1 style="font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;;">Robot Video Feed</h1>
    <img src="" id="photo" alt="Robot video Feed" style="display: block;margin-left: auto;margin-right: auto;width: 70%;">
   <script>
   window.onload = document.getElementById("photo").src = window.location.protocol + "//" + window.location.hostname + ":81/stream";
  </script>
  </body>
</html>
)rawliteral";