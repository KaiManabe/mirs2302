function chbox(obj){
    let that = obj;
    if (document.getElementById(that.id).checked == true) {
        let boxes = document.querySelectorAll('input[type="checkbox"]');

        for (let i = 0; i < boxes.length; i++) {
            boxes[i].checked = false;
        }
        document.getElementById(that.id).checked = true;
    }
}

function view(){
    let element1 = document.getElementById(yes)
    console.log(element1.checked);
    let element2 = document.getElementById(no)
    console.log(element2.checked);
    if(element1=="true"){
	    form.classList.replace("hide" , "view");
    }else if(element2=="true"){
        form.classList.replace("view" , "hide");
    }
}