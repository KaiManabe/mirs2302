// JavaScript Document

/*
表示するhtmlファイルを変更する関数
引数：表示したいhtmlファイルのiframe要素のid(文字列)
*/
function select_view_html(id){
    // 全てのiframe要素のclass名"view"を消去し、"hide"を追記
	var element = document.querySelectorAll("iframe");
	for(let i = 0; i < element.length; i++){
		element[i].classList.remove("view");
        element[i].classList.add("hide");
    }

    // 選択されたidのclass名"hide"を"view"に書き換え
    selected = document.querySelector(`#${id}`);
	selected.classList.replace("hide" , "view");
}