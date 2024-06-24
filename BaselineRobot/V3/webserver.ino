// webserver processing
// these routines provide the webserver element, with video streaming, and also the
// breakout of the
// webserver part boundary
#define PART_BOUNDARY "123456789000000000000987654321"

static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

// *********************************************************************************************
// send index page on first contact and refresh
// *********************************************************************************************
static esp_err_t index_handler(httpd_req_t *req) {
  htmlFlag = 0;  // indicate that html messages can be sent
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)htmlPage, strlen(htmlPage));
}



// *********************************************************************************************
// send video stream data
// *********************************************************************************************
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  if (webSwitch) {
    webSwitch = 0;
    if (debugSerial) {Serial.println("webSwitch reset");}
  }

  while (execGuide || execVideo) {
    //if (debugSerial) {
      delay(10);  // delay to slow/stop race contitions in testing
    //}

    // standard video processing
    if (!execGuide && execVideo) {
      // get frame buffer
      fb = esp_camera_fb_get();

      // if the frame buffer is empty, log error and report failure
      if (!fb) {
        if (debugSerial) { Serial.println("Camera capture failed"); }
        res = ESP_FAIL;
      }
      // if the frame buffer is empty, log error and report failure
      if (res == ESP_OK) {
        size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      }
      if (fb) {
        esp_camera_fb_return(fb);
        fb = NULL;
        _jpg_buf = NULL;
      } else if (_jpg_buf) {
        free(_jpg_buf);
        _jpg_buf = NULL;
      }
      if (res != ESP_OK) {
        break;
      }
    }

    // if using video guidance and streaming required, run conversion
    if (execGuide && execVideo && frameAvailable == 5) {
      int guideFrameLen = guideFrame.width * guideFrame.height * 3;
      // if (debugSerial) { Serial.printf(" %d - guideFrame length\n", guideFrameLen); }
      bool jpeg_converted = fmt2jpg(guideFrame.data, guideFrameLen, guideFrame.width, guideFrame.height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len);

      free(guideFrame.data);
      guideFrame.data = NULL;

      if (!jpeg_converted) {
        if (debugSerial) { Serial.println("JPEG compression failed"); }
        res = ESP_FAIL;
      }

      if (res == ESP_OK) {
        size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      }
      if (_jpg_buf) {
        free(_jpg_buf);
        _jpg_buf = NULL;
      }
      frameAvailable = 0;
      // schedule blob or line detection again for next frame
      if (execLine) {
        lineUpdate = 1;
      } else if (execBlob) {
        blobUpdate = 1;
      }
      if (res != ESP_OK) {
        break;
      }
    }
  }
  return res;
}

// *********************************************************************************************
// process received GET request. This primarily gets the variable and passes it to the cmdprocessor
// *********************************************************************************************
static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  //sensor_t *s = esp_camera_sensor_get();


  if (cmdProcessor(variable, 1)) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}



// *********************************************************************************************
// process received GET request for data
// *********************************************************************************************
static esp_err_t data_handler(httpd_req_t *req) {
  // return the current message
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, msg, strlen(msg));
  // if the message contained data, set data to a null type and reset the htmlFlag
  if (htmlFlag) {
    strcpy(msg, "{\"type\":null}");
    htmlFlag = 0;
  }
  return ESP_OK;
}

// *********************************************************************************************
// start the web server
// *********************************************************************************************
void startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = configWebPort;
  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/action",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  // if camera or video not active then don't init this structure
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t data_uri = {
    .uri = "/data",
    .method = HTTP_GET,
    .handler = data_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &data_uri);
  }
  // config.server_port += 1;
  config.server_port = configStreamPort;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// *********************************************************************************************
// Process HTML command
// The purpose of this command option is to provide a mechanism to send data back to the
// client browser to update the web page. It's expected that this will be in the form of
// an unbound JSON string but that's up to the user.
//
// *********************************************************************************************
int htmlProcessor(char htmlCommand[], int source) {
  // check if last html message has been sent. If it hasn't then wait for 100ms. If still hasn't
  // been sent then flag a time out and stop future message until web page is reloaded
  if (htmlFlag == 2) return 1;

  for (int i = 0; i > 99; i++) {
    if (htmlFlag) {
      delay(1);
    } else break;
  }
  if (htmlFlag) {
    htmlFlag = 2;
    if (debugSerial) { Serial.printf("HTML command timeout %s  %d \n", htmlCommand, source); }
    return 1;
  }
  if (htmlCommand[0] != '{') {
    if (source) {
      sprintf(msg, "{ %s,\"type\":\"user\"} ", htmlCommand);
    } else {
      sprintf(msg, "{ %s,\"type\":\"status\"} ", htmlCommand);
    }
  } else strcpy(msg, htmlCommand);
  htmlFlag = 1;

  return 0;
}