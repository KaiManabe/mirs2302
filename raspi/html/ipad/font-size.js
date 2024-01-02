function fontSizeAdjuster(element){
    var vw = 90.0 / element.innerHTML.length;
    element.style.fontSize = vw + "vw";
}

elements = document.querySelectorAll(".title, .description, .warn");
elements.forEach(fontSizeAdjuster);