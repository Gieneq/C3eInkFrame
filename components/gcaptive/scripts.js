const image_input = document.getElementById('image_input');
const fileInputImage = document.getElementById('fileInputImage');

layout_rotate_btn = document.getElementById('layout_rotate_btn')
layout_image = document.getElementById('layout_image')
const toggle_preview_btn = document.getElementById('toggle_preview_btn');
let preview_enable = true;

const preview_canvas = document.getElementById('preview_canvas');
preview_ctx = preview_canvas.getContext('2d');
preview_ctx.willReadFrequently = true;

const scale_slider = document.getElementById('scale_slider');
const scale_value = document.getElementById('scale_value');
let scale = 1.0;
let translation = {
  x: 0.0,
  y: 0.0
};

let dragging = {
  start_x: 0,
  start_y: 0,
  is_dragging: false
};

const img = new Image();
const target_width  = 800;
const target_height = 480;

preview_canvas.width = target_width;
preview_canvas.height = target_height;

const gamma_slider = document.getElementById('gamma_slider');
const gamma_value = document.getElementById('gamma_value');
let gamma = 1.0;

const debug_upload = false;
const upload_endpoint = "/upload_bmp";
// let upload_endpoint = "http://httpbin.org/post"; //"192.168.0.21/upload_bmp";
let final_result_data = [];

function getBaseURL() {
  // Dynamically determine the base URL using the current page's location
  const { protocol, hostname, port } = window.location;

  if (protocol === "file" || hostname === "") {
    return undefined;
  }
  
  // Construct the base URL without the port if it's the default port (80 for HTTP, 443 for HTTPS)
  const baseUrl = port === '80' || port === '443' ? `${protocol}//${hostname}` : `${protocol}//${hostname}:${port}`;
  
  return baseUrl;
}

fileInputImage.addEventListener('click', function() {
  image_input.click();
});

image_input.addEventListener('change', function(event) {
  const file = event.target.files[0];
  if (file) {
    const reader = new FileReader();
    reader.readAsDataURL(file);
    reader.onload = function(e) {
      img.src = e.target.result;
      img.onload = function() {
        console.log("Got image: ", img.width, "x", img.height);
        translation.x = img.width / 2.0;
        translation.y = img.height / 2.0;
        
        show_preview();
      };
    };
  }
});

function mapValue(value, inputMin, inputMax, outputMin, outputMax) {
  return ((value - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin) + outputMin;
}

scale_slider.addEventListener('input', function() {
  if (scale_slider.value < 50) {
    scale = mapValue(scale_slider.value, 0, 50, 0.2, 1);
  } else {
    scale = mapValue(scale_slider.value, 50, 100, 1, 4);
  }
  scale_value.textContent = scale.toFixed(2);
  show_preview();
});

gamma_slider.addEventListener('input', function() {
  if (gamma_slider.value < 50) {
    gamma = mapValue(gamma_slider.value, 0, 50, 0.5, 1);
  } else {
    gamma = mapValue(gamma_slider.value, 50, 100, 1, 2);
  }
  gamma_value.textContent = gamma.toFixed(2);
  show_preview();
});

function rgba_filter_grayscale(image_data) {
  const result = new ImageData(image_data.width, image_data.height);
  result.data.set(new Uint8ClampedArray(image_data.data));

  const result_rgba_array = result.data;
  for(let i = 0; i < result_rgba_array.length; i += 4) {
    let r = result_rgba_array[i + 0];
    let g = result_rgba_array[i + 1];
    let b = result_rgba_array[i + 2];

    let gray = 0.299 * r + 0.587 * g + 0.114 * b
    gray = Math.pow(gray, gamma);

    result_rgba_array[i + 0] = gray
    result_rgba_array[i + 1] = gray;
    result_rgba_array[i + 2] = gray;
  }
  return result;
}

function dithering(imageData, width, height) {
  const data = imageData.data;
  
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const idx = (y * width + x) * 4;
      
      const gray = data[idx];
      const new_gray = Math.round(data[idx] / 255) * 255;

      data[idx] = new_gray;
      data[idx + 1] = new_gray;
      data[idx + 2] = new_gray;
      
      const err_gray = gray - new_gray;
      distributeError(data, width, height, x, y, err_gray);
    }
  }
  return imageData;
}

function distributeError(data, width, height, x, y, err_gray) {
  // Floyd-Steinberg error distribution
  const distribute = [
    [1, 0, 7/16],
    [-1, 1, 3/16],
    [0, 1, 5/16],
    [1, 1, 1/16]
  ];
  
  for (let i = 0; i < distribute.length; i++) {
    const dx = distribute[i][0];
    const dy = distribute[i][1];
    const factor = distribute[i][2];
    
    const newX = x + dx;
    const newY = y + dy;
    
    if (newX >= 0 && newX < width && newY >= 0 && newY < height) {
      const idx = (newY * width + newX) * 4;
      
      let messed_gray = Math.max(0, Math.min(255, data[idx] + err_gray * factor));
      data[idx] = messed_gray;
      data[idx + 1] = messed_gray;
      data[idx + 2] = messed_gray;
    }
  }
}

function show_preview() {
  preview_ctx.clearRect(0, 0, preview_canvas.width, preview_canvas.height);

  const src_width = preview_canvas.width / scale;
  const src_height = preview_canvas.height / scale;
  const src_x = translation.x - src_width / 2.0;
  const src_y = translation.y - src_height / 2.0;

  preview_ctx.drawImage(img, src_x, src_y, src_width, src_height, 
    0, 0, preview_canvas.width, preview_canvas.height);

  /* Apply filter */
  const image_data = preview_ctx.getImageData(0, 0, preview_canvas.width, preview_canvas.height);
  const result_data = (preview_enable == true) ? dithering(rgba_filter_grayscale(image_data), preview_canvas.width, preview_canvas.height) : image_data;
  final_result_data = result_data.data;
  preview_ctx.putImageData(result_data, 0, 0);
};

function saveCanvasAsImage(preview_canvas, filename) {
  // Convert canvas to Data URL
  const dataURL = preview_canvas.toDataURL('image/png');
  
  // Create a link element
  const link = document.createElement('a');
  link.href = dataURL;
  link.download = filename || 'canvas-image.png';
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
}

function download() {
  const dataURL = preview_canvas.toDataURL().replace('data:', '').replace(/^.+,/, '');
  saveCanvasAsImage(preview_canvas, 'dithered_image.png');
}

// Mouse events
preview_canvas.addEventListener('mousedown', function(event) {
  dragging.is_dragging = true;
  dragging.start_x = event.clientX - preview_canvas.offsetLeft;
  dragging.start_y = event.clientY - preview_canvas.offsetTop;
});

preview_canvas.addEventListener('mousemove', function(event) {
  if (dragging.is_dragging) {
      const x = event.clientX - preview_canvas.offsetLeft;
      const y = event.clientY - preview_canvas.offsetTop;
      const deltaX = x - dragging.start_x;
      const deltaY = y - dragging.start_y;
      dragging.start_x = x;
      dragging.start_y = y;

      translation.x -= deltaX / scale;
      translation.y -= deltaY / scale;
      
      if (deltaX != 0 && deltaY != 0) {
        show_preview();
      }
  }
});

preview_canvas.addEventListener('mouseup', function() {
  dragging.is_dragging = false;
});

// Touch events
preview_canvas.addEventListener('touchstart', function(event) {
  dragging.is_dragging = true;
  if (event.type === 'mousedown') {
    dragging.start_x = event.clientX - preview_canvas.offsetLeft;
    dragging.start_y = event.clientY - preview_canvas.offsetTop;
  } else if (event.type === 'touchstart') {
    dragging.start_x = event.touches[0].clientX - preview_canvas.offsetLeft;
    dragging.start_y = event.touches[0].clientY - preview_canvas.offsetTop;
  }
  event.preventDefault(); // Prevent default behavior
});

preview_canvas.addEventListener('touchmove', function(event) {
  if (dragging.is_dragging) {
    let x, y;
    if (event.type === 'mousemove') {
      x = event.clientX - preview_canvas.offsetLeft;
      y = event.clientY - preview_canvas.offsetTop;
    } else if (event.type === 'touchmove') {
      x = event.touches[0].clientX - preview_canvas.offsetLeft;
      y = event.touches[0].clientY - preview_canvas.offsetTop;
    }
    
    const deltaX = x - dragging.start_x;
    const deltaY = y - dragging.start_y;
    
    dragging.start_x = x;
    dragging.start_y = y;

    translation.x -= deltaX / scale;
    translation.y -= deltaY / scale;

    if (deltaX !== 0 && deltaY !== 0) {
      show_preview();
    }
    event.preventDefault(); // Prevent default behavior
  }
});

preview_canvas.addEventListener('touchend', function() {
  dragging.is_dragging = false;
});

layout_rotate_btn.addEventListener('click', function() {

  if (layout_image.classList.contains('rotated')) {
    preview_canvas.width = target_width;
    preview_canvas.height = target_height;
    layout_image.classList.remove('rotated');
  } else {
    preview_canvas.width = target_height;
    preview_canvas.height = target_width;
    layout_image.classList.add('rotated');
  }
  show_preview();
});

function toggle_preview() {
  preview_enable = !preview_enable;
  toggle_preview_btn.textContent = (preview_enable == true) ? "Original" : "Preview"
  show_preview();
};

/* Uploading */
function rgba_bw_data_to_monochrome(rgba_bw_data) {
  const bw_data = [];
  for (let i = 0; i < rgba_bw_data.length; i += 4) {
    bw_data.push(rgba_bw_data[i] & 0x1);
  }
  return bw_data;
}

function monochrome_to_bmp_data(bw_data, width, height) {
  const bmp_data = [];
  for (let i = 0; i < bw_data.length; i += 8) {
    const p_y = height - Math.floor(i / width) - 1;
    const p_x = i % width;
    const p_idx = p_y * width + p_x;
    let byte_value = 0;
    for (let bit_i = 0; bit_i < 8; bit_i += 1) {
      if(bw_data[p_idx + bit_i] == 1) {
        byte_value = byte_value | (1 << ( 8 - bit_i - 1));
      }
    }
    bmp_data.push(byte_value);
  }
  return bmp_data;
}

function createBMP(monochromeData, width, height) {
  const datalen = monochromeData.length;
  const fileSize = 62 + datalen;
  const header = new Uint8Array([
      /*00*/ 0x42, 0x4D, // BM
      /*02*/ fileSize & 0xff, (fileSize >> 8) & 0xff, (fileSize >> 16) & 0xff, (fileSize >> 24) & 0xff,
      /*06*/ 0x00, 0x00, 0x00, 0x00, // Reserved
      /*0a*/ 0x3e, 0x00, 0x00, 0x00, // Offset to pixel data
      /*0e*/ 0x28, 0x00, 0x00, 0x00, // Info Header size
      /*12*/ width & 0xff, (width >> 8) & 0xff, 0x00, 0x00, // Width
      /*16*/ height & 0xff, (height >> 8) & 0xff, 0x00, 0x00, // Height
      /*1a*/ 0x01, 0x00, // Planes
      /*1c*/ 0x01, 0x00, // Bits per pixel (1-bit)
      /*1e*/ 0x00, 0x00, 0x00, 0x00, // Compression
      /*22*/ datalen & 0xff, (datalen >> 8) & 0xff, (datalen >> 16) & 0xff, (datalen >> 24) & 0xff, // Image size
      /*--*/ 0x00, 0x00, 0x00, 0x00, // X pixels per meter
      /*--*/ 0x00, 0x00, 0x00, 0x00, // Y pixels per meter
      /*--*/ 0x00, 0x00, 0x00, 0x00,
      /*--*/ 0x00, 0x00, 0x00, 0x00, 
      /*--*/ 0x00, 0x00, 0x00, 0x00,
      /*3a*/ 0xff, 0xff, 0xff, 0x00,
  ]);

  const bmpData = new Uint8Array(header.length + monochromeData.length);
  bmpData.set(header, 0);
  bmpData.set(monochromeData, header.length);

  return bmpData;
}

async function sendToEndpoint(bmpData, endpointUrl) {
  const formData = new FormData();
  const blob = new Blob([bmpData], { type: 'image/bmp' });
  formData.append('file', blob, 'image.bmp');

  const response = await fetch(endpointUrl, {
      method: 'POST',
      body: formData
  });

  if (response.ok) {
      const responseData = await response.json();
      console.log(responseData);
  } else {
      console.error('Failed to send data to the endpoint.');
  }
}

function upload() {
  if (preview_enable == false) {
    toggle_preview_btn.click();
  }
  show_preview();
  const bw_data = rgba_bw_data_to_monochrome(final_result_data);
  const bmp_data = monochrome_to_bmp_data(bw_data, preview_canvas.width, preview_canvas.height);
  const bmp_file_data = createBMP(bmp_data, preview_canvas.width, preview_canvas.height);

  const base_url = getBaseURL();

  if (debug_upload || (base_url == undefined)) {
    console.log("Donwload mode of", bmp_file_data.length, "B");

    const blob = new Blob([bmp_file_data], { type: 'image/bmp' });
    const url = URL.createObjectURL(blob);

    const link = document.createElement('a');
    link.href = url;
    link.download = 'canvas-image.bmp';

    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  } else {
    const full_url = `${base_url}${upload_endpoint}`;
    console.log("Upload mode of", bmp_file_data.length, "B to", full_url);
    
  // console.log(header);
    sendToEndpoint(bmp_file_data, full_url);
  }
}

// Used to toggle the menu on smaller screens when clicking on the menu button
function openNav() {
    var x = document.getElementById("navDemo");
    if (x.className.indexOf("w3-show") == -1) {
      x.className += " w3-show";
    } else { 
      x.className = x.className.replace(" w3-show", "");
    }
  }