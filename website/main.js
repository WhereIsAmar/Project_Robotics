// Runs on page load
// Setup all event listeners here.
function onPageLoad() {
    //save
    document.querySelectorAll('.js-playback')
        .forEach(btn => btn.addEventListener('click', playback));
}

function playback(event) {
    const name = event.target.textContent;
    fetch(`http://localhost:8080/playback/${name}`, {mode: 'no-cors'})
        .then(() => alert('done'));
}


window.addEventListener('DOMContentLoaded', onPageLoad);
