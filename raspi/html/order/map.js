function showImage(imageSrc) {
    document.getElementById('overlay-img').src = imageSrc;
    document.getElementById('overlay').style.display = 'block';
    document.getElementById('image-container').style.display = 'block';
    document.getElementById('close-button').style.display = 'block';
    document.getElementById('overlay-img').style.display = 'block';
}

function hideImage() {
    document.getElementById('overlay').style.display = 'none';
    document.getElementById('image-container').style.display = 'none';
    document.getElementById('close-button').style.display = 'none';
    document.getElementById('overlay-img').style.display = 'none';
}