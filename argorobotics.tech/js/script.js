
var slideIndex = [1, 1, 1]; // Adjusted for three slideshows

function plusSlides(n, no) {
  showSlides(slideIndex[no] += n, no);
}

function showSlides(n, no) {
  var slides = document.getElementsByClassName("slideshow-container")[no].getElementsByClassName("mySlides");
  if (n > slides.length) {slideIndex[no] = 1}
  if (n < 1) {slideIndex[no] = slides.length}
  for (var i = 0; i < slides.length; i++) {
     slides[i].style.display = "none";  
  }
  slides[slideIndex[no]-1].style.display = "block";  
}

// Initialize slideshows
showSlides(1, 0);
showSlides(1, 1);
showSlides(1, 2); // Added initialization for the third slideshow
showSlides(1, 3);
showSlides(1, 4);
showSlides(1, 5); // Added initialization for fifth slideshow

// Optional: Add auto-switching functionality
function autoSwitch() {
  for (var no = 0; no < slideIndex.length; no++) {
    plusSlides(1, no);
  }
  setTimeout(autoSwitch, 1000); // Change image every 4 seconds
}

autoSwitch(); // Start the auto-switching function
