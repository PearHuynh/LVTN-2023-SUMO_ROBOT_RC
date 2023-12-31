#ifndef __CSS_PAGE_H
#define __CSS_PAGE_H

#include <Arduino.h>

    //-----HTML-----
    const char Main_Page[] PROGMEM = R"-----(
    <html>

    <head>
        <!------------------------------------------CSS----------------------------------------------------->
        <style>
            /* Body style */
            body {
                font: 100%/125% 'Times New Roman', Times, serif;
                margin-right: 10%;
                margin-left: 10%;
                padding: 0px;
                background-color: rgb(226, 233, 232);
            }

            /* Wrapper style */
            #wrapper {
                margin: 10px;
                padding: 0px;
                width: 100%;
                border-radius: 8px;
            }

            /* Header style */
            #header {
                display: flex;
                padding: 10px;
                flex-wrap: wrap;
                /* justify-content: space-between; */
                justify-content: center;
                align-items: center;
                line-height: 2.2em;
                border-radius: 10px;
                background-color: gray;
                color: white;
            }

            #logo_header {
                padding-left: 10%;
                padding-right: 10%;
                /* margin-left: 10%; */
                background-color: lightslategray;
                border-radius: 10px;
                box-shadow: 15px 8px 8px rgba(12, 6, 6, 0.5);
            }

            #logo_header h1 {
                font-size: 35px;
                color: white;
                font-family: 'Open Sans', sans-serif;
            }

            #lish_header {
                margin-right: 5%;
                margin-left: 5%;
            }

            #lish_header ul {
                display: flex;
            }

            #lish_header li {
                list-style: none;
                margin-right: 10px;
            }

            #lish_header a {
                text-decoration: none;
                font-size: 26px;
                color: white;
                font-weight: bold;
                padding: 10px;
                background-color: lightslategray;
                box-shadow: 5px 5px 10px rgba(12, 6, 6, 0.5);
            }

            #lish_header li a:hover:not(.active) {
                color: black;
                border-radius: 8px;
            }

            #lish_header li a.active {
                color: black;
                cursor: auto;
                /* text-decoration: underline; */
            }

            /* main-container style */
            #main-container {
                display: flex;
                flex-wrap: wrap;
                justify-content: space-evenly;
                /* Sử dụng space-between để đặt các khối cách xa nhau */
                /* align-items: center; */
                align-items: stretch;
                /* background-color: darkgray; */
                border-radius: 10px;
                margin: 10px auto;
            }

            .box-container {
                flex: 0 0 calc(50% - 10px);
                /* Sử dụng flex để điều chỉnh kích thước của các khối và loại bỏ margin và padding */
                justify-content: center;
                align-items: center;
                background-color: azure;
                border: 3px solid #333;
                border-radius: 10px;
                margin: 10px 0;
                /* Loại bỏ margin auto và thêm margin trên và dưới */
                padding: 10px;
                box-sizing: border-box;
            }

            .box-container h1 {
                padding: 15px;
                margin-top: 5px;
                margin-left: 10px;
                text-align: center;
                color: black;
                font-size: 35px;
                border-radius: 5px;
            }

            .box-container h2 {
                color: black;
                margin-left: 10px;
                font-size: 20px;
            }

            /*Control GPIO input/output, box left*/
            .gpio-container {
                margin-left: 30px;
            }

            .select_option {
                padding: 2px;
                margin-left: 30px;
                font-size: 15px;
            }

            .span_option {
                margin-left: 30px;
            }

            .bt_on_off {
                margin-left: 30px;
                opacity: 0.8;
            }

            /*Control SPEED motor left and right, box right*/
            .select_option_control {
                padding: 2px;
                text-align: center;
                padding: 5px;
                margin-left: 30px;
                width: 30%;
                font-size: 15px;
            }

            #setting_speed {
                opacity: 0.5;
                pointer-events: none;
            }


            /*slider control speed, box right*/
            #motor_control {
                flex: 1;
                padding: 20px;
                background-color: #cbe2e6;
                border-radius: 5px;
            }

            #motor_control input[type="range"] {
                width: 80%;
                /* Độ rộng của thanh slider */
                display: block;
                margin: 0 auto;
                /* Để thanh slider nằm giữa */
            }

            .checkbox_control {
                margin-left: 30px;
                width: 15px;
                height: 15px;
            }

            .view_speed {
                display: inline-block;
                width: 50%;
            }

            /*this is css setting speed, box right*/
            #setting_speed {
                flex: 1;
                padding: 20px;
                background-color: #cbe2e6;
                border-radius: 5px;
            }

            .bt_set_speed {
                background-color: aquamarine;
                border-color: white;
                color: red;
                border-radius: 5px;
                margin-left: 10px;
            }

            #bt_start {
                background-color: aquamarine;
                border-color: white;
                color: red;
                border-radius: 5px;
                margin-left: 10px;
            }

            #setting_speed input[type="range"] {
                width: 50%;
                /* Độ rộng của thanh slider */
                display: inline-block;
                margin: 0 auto;
                /* Để thanh slider nằm giữa */
            }
        </style>
    </head>

    <!------------------------------------------BODY----------------------------------------------------->

    <body>
        <div id="wrapper">
            <div id="header">
                <div id="logo_header">
                    <h1>ESP32 Control Panel</h1>
                </div>
                <!-- <div id="lish_header">
                    <ul>
                        <li><a href="#">Home</a></li>
                        <li><a href="#" class="active">Issues</a></li>
                        <li><a href="#">Contact</a></li>
                    </ul>
                </div> -->
            </div>

            <div id="main-container">
                <div class="box-container">
                    <h1>GPIO control panel</h1>
                    <div class="gpio-container" id="gpio-control"></div>
                </div>

                <div class="box-container">
                    <h1>Control speed</h1>

                    <h2 style="display: inline-block;">Motor Control</h2>
                    <input type="checkbox" checked id="checkbox_motor_control" style="width: 15px; height: 15px;">

                    <div id="motor_control">
                        <p class="view_speed"> Motor left speed:
                            <span id="motor_speed_left">0</span> PWM
                        </p>
                        <input type="checkbox" checked id="c_fwd_left" class="checkbox_control"> FWD
                        <input type="checkbox" id="c_rev_left" class="checkbox_control"> REV
                        <input type="range" id="motor_speed_slider_left" min="0" max="1023" step="1">
                        <br>
                        <p class="view_speed">Motor right speed:
                            <span id="motor_speed_right">0</span> PWM
                        </p>
                        <input type="checkbox" checked id="c_fwd_right" class="checkbox_control"> FWD
                        <input type="checkbox" id="c_rev_right" class="checkbox_control"> REV
                        <input type="range" id="motor_speed_slider_right" min="0" max="1023" step="1">
                        <br>
                    </div>

                    <br>

                    <h2 style="display: inline-block;"> Setting Speed</h2>
                    <input type="checkbox" id="checkbox_setting_speed" style="width: 15px; height: 15px;">

                    <div id="setting_speed">
                        <p class="view_speed"> Motor left speed: <span id="s_motor_speed_left">0</span> PWM</p>
                        <input type="checkbox" checked id="set_fwd_left" class="checkbox_control"> FWD
                        <input type="checkbox" id="set_rev_left" class="checkbox_control"> REV
                        <br>
                        <label>Set PWM:</label>
                        <input type="number" id="in_set_speed_left" style="width: 20%;">
                        <button id="bt_set_speed_left" class="bt_set_speed">SET</button>

                        <br>
                        <p class="view_speed">Motor right speed: <span id="s_motor_speed_right">0</span> PWM</p>
                        <input type="checkbox" checked id="set_fwd_right" class="checkbox_control"> FWD
                        <input type="checkbox" id="set_rev_right" class="checkbox_control"> REV

                        <br>
                        <label>Set PWM:</label>
                        <input type="number" id="in_set_speed_right" style="width: 20%;">
                        <button id="bt_set_speed_right" class="bt_set_speed">SET</button>

                        <br><br>
                        <label>Set Delay:</label>
                        <input type="number" id="in_set_delay" style="width: 20%;">
                        <button id="bt_start">START</button>
                        <br><br>
                    </div>

                    <div>

                    </div>

                </div>

            </div>

            <div id="footer">
                <p>&copy;2023 Design and Manufacture of Sumo Robot. Control Engineering and Automation Thesis.</p>
            </div>
        </div>

        <!------------------------------------------SCRIPT----------------------------------------------------->

        <script>
            var state_control_speed;

            //tao doi tuong request
            function create_obj() {
                var obj;
                if (window.XMLHttpRequest) {
                    obj = new XMLHttpRequest();
                } else if (window.ActiveXObject) {
                    try {
                        obj = new ActiveXObject("Microsoft.XMLHTTP");
                    } catch (e) {
                        obj = null;
                    }
                }
                return obj;
            }
            var xhttp = create_obj();

            /*----------------------------------------BOX LEFT---------------------------------------------------*/

            //Tao html hien thi cac gpio INPUT/OUPUT
            function createGPIOItem(pinNumber) {
                const listItem = document.createElement("div");
                listItem.innerHTML = `
                 <label>GPIO ${pinNumber}:</label>
                 <select id="gpio_${pinNumber}" class="select_option">
                 <option value="input" selected>Input</option>
                 <option value="output">Output</option>
                 </select>
                 <span id="gpio_status_${pinNumber}" class="span_option">Status: 0</span>
                 <button id="on_off_${pinNumber}" class="bt_on_off" disabled>ON/OFF</button>
                 <span id="gpio_set_${pinNumber}" class="span_option">Set: IN</span>
                 <br><br>
             `;
                return listItem;
            }

            //Tao html hien thi cac gpio INPUT
            function create_In_Item(pinNumber) {
                const listItem = document.createElement("div");
                listItem.innerHTML = `
            <label>GPIO ${pinNumber}:</label>
                 <select id="gpio_${pinNumber}" class="select_option">
                 <option value="input" selected>Input</option>
                 </select>
                 <span id="gpio_status_${pinNumber}" style="margin-left: 30px;">Status: 0</span>
                 <button id="on_off_${pinNumber}" style="display: none;"></button>
                 <br><br>
             `;
                return listItem;
            }

            //Doan chuong trinh thu hien cho cac Element phia ben box left ----------------------------------------------
            function addGPIOItems(pinNumber) {
                const gpioContainer = document.getElementById("gpio-control");
                var listItem;

                // Tạo tiêu đề và thêm vào giao diện trước các mục GPIO
                if (pinNumber === 13) {
                    const header = document.createElement("h3");
                    header.textContent = "GPIO INPUT/OUTPUT";
                    gpioContainer.appendChild(header);
                } else if (pinNumber === 34) {
                    const header = document.createElement("h3");
                    header.textContent = "GPIO INPUT";
                    gpioContainer.appendChild(header);
                }

                //Tao 2 khu vuc hien thi cac gpio co the in/out va cac gpio chi co the out
                if (pinNumber <= 33) {
                    listItem = createGPIOItem(pinNumber);
                } else {
                    listItem = create_In_Item(pinNumber);
                }

                //gpio 39 doc DIP SW nen doc ADC
                if (pinNumber === 39) {
                    listItem.querySelector(`span`).textContent = "Read ADC: 0";
                }

                //add vao dia chi id tai vi tri dat truoc
                gpioContainer.appendChild(listItem);

                //kiem tra trang thai chan IO
                var JSON_receiver_IO;
                function getStatusLED() {
                    xhttp.open("GET", "/gpio_status", true);
                    xhttp.onreadystatechange = function () {
                        if (xhttp.readyState === 4) {
                            if (xhttp.status === 200) {
                                try {
                                    var receive_from_esp32 = xhttp.responseText;
                                    var JsonData = JSON.parse(receive_from_esp32);
                                    console.log(JsonData);
                                    JSON_receiver_IO = JsonData;
                                    view_status(JsonData);
                                    view_set_status(JsonData);
                                    state_control_speed = JsonData.CS; //Lay trang thai dieu khien bien CS Json
                                } catch (error) {
                                    console.error("Lỗi phân tích chuỗi JSON: " + error);
                                    // Xử lý lỗi ở đây nếu cần
                                }
                            } else {
                                console.error("Lỗi yêu cầu: Mã trạng thái " + xhttp.status);
                                // Xử lý lỗi ở đây nếu cần
                            }
                        }
                    };
                    xhttp.send('{}');
                }
                //Set tu dong goi kiem tra trang thai sau 100ms
                setInterval(function () {
                    getStatusLED();
                }, 100);

                const onOffButton = document.getElementById(`on_off_${pinNumber}`);
                const gpioSelect = document.getElementById(`gpio_${pinNumber}`);
                const statusElement = document.getElementById(`gpio_status_${pinNumber}`);
                var st_button = 0;

                //Kiem tra trang thai mode o che do ngo vao hay ngo ra
                gpioSelect.addEventListener("change", function () {
                    if (gpioSelect.value === "input") {
                        onOffButton.style.opacity = "0.8"; // Nút trở mờ khi chọn "Input"
                        onOffButton.disabled = true;

                        xhttp.open("POST", "/receive_data", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        for (let i = 0; i < 3; i++) {
                            xhttp.send(`{"IO": ${pinNumber}, "ST": 0}`);
                        }
                    } else {
                        onOffButton.style.opacity = "1"; // Khôi phục độ trong suốt khi chọn "Output"
                        onOffButton.disabled = false;

                        xhttp.open("POST", "/receive_data", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        // xhttp.onreadystatechange = function () {
                        //     if (xhttp.readyState === 4 && xhttp.status === 200) {
                        //         onOffButton.style.opacity = "1"; // Khôi phục độ trong suốt khi chọn "Output"
                        //         onOffButton.disabled = false;
                        //     } 
                        // };
                        for (let i = 0; i < 3; i++) {
                            xhttp.send('{"IO": ' + pinNumber + ', "ST": 1}');
                        }
                    }
                });

                //Kiem tra trang thai nut nhan o che do output set out muc HIGH/LOW
                onOffButton.addEventListener("click", function () {
                    st_button ^= 1;

                    xhttp.open("POST", "/receive_status", true);
                    xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                    xhttp.send(JSON.stringify({ IO: pinNumber, ST: st_button }));
                });

                //hien thi trang thai muc logic cac gpio
                function view_status(dataReceiver) {
                    const gpioList = [13, 14, 16, 17, 18, 19, 32, 33, 34, 35, 36, 39];

                    gpioList.forEach((gpio) => {
                        const element = document.getElementById(`gpio_status_${gpio}`);
                        if (element) {
                            element.innerHTML = `Status: ${dataReceiver['ST' + gpio]}`;
                        }
                    });
                }

                //Hien thi trang thai set mode IN/OUT
                function view_set_status(dataReceiver) {
                    const gpioList = [13, 14, 16, 17, 18, 19, 32, 33];

                    gpioList.forEach((gpio) => {
                        const element = document.getElementById(`gpio_set_${gpio}`);
                        if (element) {
                            element.innerHTML = `Set mode: ${dataReceiver['S' + gpio]}`;
                        }
                    });
                }
            }

            // Add GPIO items when the page loads
            window.addEventListener("load", function () {
                // Add GPIO items when the page loads
                [13, 14, 16, 17, 18, 19, 32, 33, 34, 35, 36, 39].forEach((pin) => {
                    addGPIOItems(pin);
                });
            });


            /*----------------------------------------BOX RIGHT---------------------------------------------------*/

            //Doan chuong trinh thu hien cho cac Element phia ben box right
            document.addEventListener("DOMContentLoaded", function () {
                const checkbox_motor_control = document.getElementById("checkbox_motor_control");
                const checkbox_setting_speed = document.getElementById("checkbox_setting_speed");
                var motor_control = document.getElementById("motor_control");
                var setting_speed = document.getElementById("setting_speed");

                //Kiem tra checkbox mode 0
                checkbox_motor_control.addEventListener("change", function () {
                    if (checkbox_motor_control.checked) {
                        motor_control.style.opacity = "1";
                        motor_control.style.pointerEvents = "auto";
                        setting_speed.style.opacity = "0.5";
                        setting_speed.style.pointerEvents = "none";
                        checkbox_setting_speed.checked = false;
                       
                        xhttp.open("POST", "/mode_speed", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        // xhttp.onreadystatechange = function () {
                        //     if (xhttp.readyState === 4 && xhttp.status === 200) {
                        //         var receive_from_esp32 = xhttp.responseText;
                        //         var JsonData = JSON.parse(receive_from_esp32);
                        //         // console.log(JsonData);
                        //         if (JsonData.CS == 0) {
                                    
                        //         }
                        //     }
                        // };
                        xhttp.send(`{"MD": 0}`);
                        xhttp.send(`{"MD": 0}`);
                        xhttp.send(`{"MD": 0}`);
                    }
                });

                //Kiem tra checkbox mode 1
                checkbox_setting_speed.addEventListener("change", function () {
                    if (checkbox_setting_speed.checked) {
                        setting_speed.style.opacity = "1";
                        setting_speed.style.pointerEvents = "auto";
                        motor_control.style.opacity = "0.5";
                        motor_control.style.pointerEvents = "none";
                        checkbox_motor_control.checked = false;
                                    
                        xhttp.open("POST", "/mode_speed", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        // xhttp.onreadystatechange = function () {
                        //     if (xhttp.readyState === 4 && xhttp.status === 200) {
                        //         var receive_from_esp32 = xhttp.responseText;
                        //         var JsonData = JSON.parse(receive_from_esp32);
                        //         // console.log(JsonData);
                        //         if (JsonData.CS == 1) {

                        //         }
                        //     }
                        // };
                        xhttp.send(`{"MD": 1}`);
                        xhttp.send(`{"MD": 1}`);
                        xhttp.send(`{"MD": 1}`);
                    }
                });

                //Doan chuong trinh thuc hien dieu huong motor left
                const c_fwd_left = document.getElementById("c_fwd_left");
                const c_rev_left = document.getElementById("c_rev_left");
                const c_fwd_right = document.getElementById("c_fwd_right");
                const c_rev_right = document.getElementById("c_rev_right");

                //Kiem tra chay thuan motor trai
                c_fwd_left.addEventListener("change", function () {
                    if (c_fwd_left.checked) {
                        c_rev_left.checked = false;

                        xhttp.open("POST", "/dir_left", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"ML": 1}`);
                    }
                    if (c_fwd_left.checked == false && c_rev_left.checked == false) {
                        xhttp.open("POST", "/dir_left", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"ML": -1}`);
                    }
                });

                //Kiem tra chay nghich motor trai
                c_rev_left.addEventListener("change", function () {
                    if (c_rev_left.checked) {
                        c_fwd_left.checked = false;

                        xhttp.open("POST", "/dir_left", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"ML": 0}`);
                    }
                    if (c_fwd_left.checked == false && c_rev_left.checked == false) {
                        xhttp.open("POST", "/dir_left", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"ML": -1}`);
                    }
                });

                //Kiem tra chay thuan motor phai
                c_fwd_right.addEventListener("change", function () {
                    if (c_fwd_right.checked) {
                        c_rev_right.checked = false;

                        xhttp.open("POST", "/dir_right", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"MR": 1}`);
                    }
                    if (c_fwd_right.checked == false && c_rev_right.checked == false) {
                        xhttp.open("POST", "/dir_right", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"MR": -1}`);
                    }
                });

                //Kiem tra chay nghich motor trai
                c_rev_right.addEventListener("change", function () {
                    if (c_rev_right.checked) {
                        c_fwd_right.checked = false;

                        xhttp.open("POST", "/dir_right", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"MR": 0}`);
                    }
                    if (c_fwd_right.checked == false && c_rev_right.checked == false) {
                        xhttp.open("POST", "/dir_right", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"MR": -1}`);
                    }
                });

                //Doan chuong trinh thuc hien dieu chinh toc do thong qua slider
                const slider_left = document.getElementById('motor_speed_slider_left');
                const sliderDisplay_left = document.getElementById('motor_speed_left');
                const slider_right = document.getElementById('motor_speed_slider_right');
                const sliderDisplay_right = document.getElementById('motor_speed_right');
                slider_left.value = 0;
                slider_right.value = 0;

                //Slider speed left
                slider_left.addEventListener('input', function () {
                    const sliderValue = slider_left.value;
                    sliderDisplay_left.textContent = sliderValue;

                    xhttp.open("POST", "/receiver_speed_left", true);
                    xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                    // xhttp.onreadystatechange = function () {
                    //     if (xhttp.readyState === 4 && xhttp.status === 200) {
                    //         // var receive_from_esp32 = xhttp.responseText;
                    //         // var JsonData = JSON.parse(receive_from_esp32);
                    //         // console.log(JsonData);
                    //     }
                    // };
                    xhttp.send(`{"SPL": ${sliderValue}}`);
                });

                //Slider speed right
                slider_right.addEventListener('input', function () {
                    const sliderValue = slider_right.value;
                    sliderDisplay_right.textContent = sliderValue;

                    xhttp.open("POST", "/receiver_speed_right", true);
                    xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                    // xhttp.onreadystatechange = function () {
                    //     if (xhttp.readyState === 4 && xhttp.status === 200) {
                    //         // var receive_from_esp32 = xhttp.responseText;
                    //         // var JsonData = JSON.parse(receive_from_esp32);
                    //         // console.log(JsonData);
                    //     }
                    // };
                    xhttp.send(`{"SPR": ${sliderValue}}`);
                });

                //Doan chuong trinh thuc hien dieu huong motor left
                const s_fwd_left = document.getElementById("set_fwd_left");
                const s_rev_left = document.getElementById("set_rev_left");
                const s_fwd_right = document.getElementById("set_fwd_right");
                const s_rev_right = document.getElementById("set_rev_right");

                //Kiem tra chay thuan motor trai
                s_fwd_left.addEventListener("change", function () {
                    if (s_fwd_left.checked) {
                        s_rev_left.checked = false;

                        xhttp.open("POST", "/dir_left", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"ML": 1}`);
                    }
                    if (s_fwd_left.checked == false && s_rev_left.checked == false) {
                        xhttp.open("POST", "/dir_left", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"ML": -1}`);
                    }
                });

                //Kiem tra chay nghich motor trai
                s_rev_left.addEventListener("change", function () {
                    if (s_rev_left.checked) {
                        s_fwd_left.checked = false;

                        xhttp.open("POST", "/dir_left", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"ML": 0}`);
                    }
                    if (s_fwd_left.checked == false && s_rev_left.checked == false) {
                        xhttp.open("POST", "/dir_left", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"ML": -1}`);
                    }
                });

                //Kiem tra chay thuan motor phai
                s_fwd_right.addEventListener("change", function () {
                    if (s_fwd_right.checked) {
                        s_rev_right.checked = false;

                        xhttp.open("POST", "/dir_right", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"MR": 1}`);
                    }
                    if (s_fwd_right.checked == false && s_rev_right.checked == false) {
                        xhttp.open("POST", "/dir_right", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"MR": -1}`);
                    }
                });

                //Kiem tra chay nghich motor phai
                s_rev_right.addEventListener("change", function () {
                    if (s_rev_right.checked) {
                        s_fwd_right.checked = false;

                        xhttp.open("POST", "/dir_right", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"MR": 0}`);
                    }
                    if (s_fwd_right.checked == false && s_rev_right.checked == false) {
                        xhttp.open("POST", "/dir_right", true);
                        xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                        xhttp.send(`{"MR": -1}`);
                    }
                });

                //Doan chuong trinh set toc do and delay motor setting speed
                const in_set_speed_left = document.getElementById("in_set_speed_left");
                const in_set_speed_right = document.getElementById("in_set_speed_right");
                const s_motor_speed_left = document.getElementById("s_motor_speed_left");
                const s_motor_speed_right = document.getElementById("s_motor_speed_right");
                const in_set_delay = document.getElementById("in_set_delay");
                const bt_set_speed_left = document.getElementById("bt_set_speed_left");
                const bt_set_speed_right = document.getElementById("bt_set_speed_right");
                const bt_start = document.getElementById("bt_start");
               
                bt_set_speed_left.addEventListener('click', function () {
                    s_motor_speed_left.textContent = in_set_speed_left.value; 
                    xhttp.open("POST", "/receiver_speed_left", true);
                    xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                    xhttp.send(`{"SPL": ${in_set_speed_left.value}}`);
                    xhttp.send(`{"SPL": ${in_set_speed_left.value}}`);
                    xhttp.send(`{"SPL": ${in_set_speed_left.value}}`);
                });

                bt_set_speed_right.addEventListener('click', function () {
                    s_motor_speed_right.textContent = in_set_speed_right.value; 
                    xhttp.open("POST", "/receiver_speed_right", true);
                    xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                    xhttp.send(`{"SPR": ${in_set_speed_right.value}}`);
                    xhttp.send(`{"SPR": ${in_set_speed_right.value}}`);
                    xhttp.send(`{"SPR": ${in_set_speed_right.value}}`);
                });

                bt_start.addEventListener('click', function(){
                    xhttp.open("POST", "/start_motor", true);
                    xhttp.setRequestHeader("Content-Type", "application/json;charset=UTF-8");
                    xhttp.send(`{"DL": ${in_set_delay.value}}`);
                    xhttp.send(`{"DL": ${in_set_delay.value}}`);
                    xhttp.send(`{"DL": ${in_set_delay.value}}`);
                });

            });

        </script>
    </body>

    </html>
    )-----";

    #endif