/*
let slideIndex = [1, 1, 1]; // An array to hold the slide index for each carousel
const carouselClasses = ["softwareTeam", "hardwareTeam", "outreachTeam"]; // Class names for each carousel

// Initialize all carousels
carouselClasses.forEach((className, i) => showSlides(slideIndex[i], i));

function plusSlides(n, carouselNo) {
  slideIndex[carouselNo] += n;
  showSlides(slideIndex[carouselNo], carouselNo);
}

// function showSlides(n, carouselNo) {
//   let i;
//   let slides = document.getElementsByClassName(carouselClasses[carouselNo]);
//   console.log("SLIDE INDEX", slideIndex, "SLIDES", slides);
//   if (n > slides.length) {
//     slideIndex[carouselNo] = 1;
//   }
//   if (n < 1) {
//     slideIndex[carouselNo] = slides.length;
//   }
//   for (i = 0; i < slides.length; i++) {
//     slides[i].style.display = "none";
//   }
//   slides[slideIndex[carouselNo] - 1].style.display = "block";
// }

function showSlides(n, carouselNo) {
  console.log("n:", n, "carousel #", carouselNo); // This is just for testing (LOOK AT THIS)
  let slides = document.getElementsByClassName(carouselClasses[carouselNo])[0]; // Get HTML carousel element by className getElementsByClassName returns an array, so you need to get the first element of the array
  let images = slides.getElementsByClassName("my-slide"); // slides references the parent <div> object of the images. But you want the images, so you get the images

  // Go through all the images and set the styling
  for (let i = 0; i < images.length; i++) {
    if (i == n) images[i].style.display = "block";
    else images[i].style.display = "none";
  }
}
*/
// You may need to adjust the interval setup for each carousel if needed

var slideIndex = [1, 1];
var slideId = ["mySlides1", "mySlides2"]
showSlides(1, 0);
showSlides(1, 1);

function plusSlides(n, no) {
  showSlides(slideIndex[no] += n, no);
}

function showSlides(n, no) {
  var i;
  var x = document.getElementsByClassName(slideId[no]);
  if (n > x.length) {slideIndex[no] = 1}    
  if (n < 1) {slideIndex[no] = x.length}
  for (i = 0; i < x.length; i++) {
     x[i].style.display = "none";  
  }
  x[slideIndex[no]-1].style.display = "block";  
}

// Optional: Add auto-switching functionality
function autoSwitch() {
  for (var no = 0; no < slideId.length; no++) {
    plusSlides(1, no);
  }
  setTimeout(autoSwitch, 4000); // Change image every 4 seconds
}

autoSwitch(); // Start the auto-switching function
