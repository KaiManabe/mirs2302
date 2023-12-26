// JavaScript Document

/*
表示するhtmlファイルを変更する関数
引数：表示したいhtmlファイルのiframe要素のid(文字列)
*/
function select_view_html(iframe_id){
    // 全てのiframe要素のclass名"view"を消去し、"hide"を追記
	var iframeElement = document.querySelectorAll("iframe");
	for(let i = 0; i < iframeElement.length; i++){
		iframeElement[i].classList.remove("view");
        iframeElement[i].classList.add("hide");
    }

    // 選択されたidのclass名"hide"を"view"に書き換え
    selected = document.querySelector(`#${iframe_id}`);
	selected.classList.replace("hide" , "view");
}


// ナビゲーションメニューのリストアイテムを取得
var navItems = document.querySelectorAll('nav ul li div');
// 各リストアイテムにクリックイベントを追加
navItems.forEach(function(item) {
  item.addEventListener('click', function() {
    // すべてのリストアイテムからクラスを削除
    navItems.forEach(function(item) {
      item.classList.remove('selected');
    });

    // クリックされたリストアイテムにクラスを追加
    item.classList.add('selected');
  });
});

