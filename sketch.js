var points = []; //T1,T2,T3,T4,Tp
var triangulation = [];
var hull = [];

function setup() {
  createCanvas(1200, 800); //canvas na katerega rišemo
  input = createInput(); //vnos x
  input.position(15,850); //kje se nahaja
  button1 = createButton('Even'); //gumb 
  button1.position(15,870);
  button1.mousePressed(even_dist);
  button2 = createButton('Gauss'); //gumb 
  button2.position(15,890);
  button2.mousePressed(gauss_dist);
  button3 = createButton('Run'); //gumb 
  button3.position(15,910);
  button3.mousePressed(generate_triangulation);
  radio = createRadio();
  radio.option('Greedy');
  radio.option('Hamilton');
  button3.position(115,870);
}

function create_point(p){ //ustvarimo točko
  points.push(p);
  strokeWeight(4);
  stroke('red');
  point(p.x,p.y);
}

function equals(a, b) {
  return abs(a - b) < 1e-9;
}

function euk_dist(a,b){
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

function grahangle(a,b){
  var a =atan2(a.x-b.x,a.y-b.y);
  if(a < 0){
    a = a + 2*PI;
  }
  return a;
}

function grahamsort(a,b){
  if(grahangle(b,points[0]) < grahangle(a,points[0])){
    return 1;
  }
  else if(equals(grahangle(b,points[0]),grahangle(a,points[0]))){
    if(euk_dist(points[0],a) > euk_dist(points[0],b)){
      return 1;
    }
  }
  return -1;
}

function grahams_scan(){
  hull = [];
  points.sort((a, b) => (a.x > b.x) ? 1 : -1);
  var temp1 = points.slice(1);
  var temp2 = points.slice(0, 1);
  temp1.sort(grahamsort);
  points = temp2.concat(temp1); 
  hull.push(points[0].copy());
  hull.push(points[1].copy());
  hull.push(points[2].copy());

  for (var i = 3; i < points.length; i++){
    var p1 = hull[hull.length-2].copy();
    var p2 = hull[hull.length-1].copy();
    var p3 = points[i].copy();
    var U = p5.Vector.cross(p3.copy().sub(p1), (p2.copy().sub(p1))).z;
    if(U <= 0){
      while(hull.length > 1 && p5.Vector.cross(points[i].copy().sub(hull[hull.length-2]), ( hull[hull.length-1].copy().sub(hull[hull.length-2]))).z <= 0){
        hull.pop();
      }
    }
    hull.push(p3);
  }
}

function generate_triangulation(){
  triangulation = [];
  hull = [];
  lines = [];
  strokeWeight(2);
  var r = radio.value();
  if(r == 'Greedy'){
    greedy_triangulation();
  }
  else if(r == 'Hamilton'){
    hamilton_triangulation();
  }
}

function jarvis_march(special_points){
  let E = points[0];
  let a = 0;
  for (var i = 1; i < points.length; i++ ){
    if(E.y > points[i].y){
      E = points[i];
    }
  }
  hull.push(E);
  points.splice(points.indexOf(E), 1);
  var s1;
  var minangle = PI;
  for (var i = 0; i < points.length; i++){
    var newangle= acos(createVector(1,0).dot(points[i].copy().sub(E)) / (points[i].copy().sub(E).mag()));
    if(newangle < minangle){
      s1 = points[i];
      minangle = newangle;
    }
    else if(equals(newangle, minangle)){
      if(euk_dist(E,s1) > euk_dist(E,points[i])){
        s1 = points[i]; 
      }
    }
  }
  hull.push(s1);
  points.splice(points.indexOf(s1),1);
  points.push(E);

  var i;
  let tmp = true;
  while(points.length>0){
    minangle = PI;
    var A = p5.Vector.sub(hull[hull.length-1], hull[hull.length-2]);
    for (var j = 0; j < points.length; j++){
      var B = p5.Vector.sub(points[j], hull[hull.length-1]);
      var ang = acos(A.copy().dot(B) / (A.mag() * B.mag()));
      if(ang < minangle){
        minangle = ang;
        i = j;
      }
    }
    if(points[i].equals(E)){
      a = (hull.length-1);
      points.splice(i,1);
      continue;
    }
    hull.push(points[i].copy());
    points.splice(i,1);
  }
  return a;
}

function hamilton_triangulation(){
  hull = [];
  let lines = [];
  let N = points.length;
  let sp = jarvis_march();
  stroke('purple');

  for (var i = 1; i < hull.length; i++){
    line(hull[i-1].x, hull[i-1].y, hull[i].x, hull[i].y);
    lines.push({a:hull[i-1],b:hull[i]});
  }
  stroke('black');
  let a = sp;
  let b = sp+1;
  let c = 0;

  triangle(hull[a].x,hull[a].y,hull[b].x,hull[b].y,hull[c].x,hull[c].y);
  lines.push({a:hull[b],b:hull[c]});
  let l = 0;
  while(l < 3*N-sp-3){
    let temp = b;
    a = b;
    b = c;
    c = min(temp + 1, N-1);
    if(check_crossing({a:hull[b],b:hull[c]},lines)){
      line(hull[b].x,hull[b].y,hull[c].x,hull[c].y);
      lines.push({a:hull[b],b:hull[c]});
    }
    else{
      c = a;
    }
    l++;
  }

  /*for(let i=0; i< 100; i++){
    let temp = {a:hull[b],b:hull[c]};
    if(check_crossing(temp,lines)){
      line(hull[b].x,hull[b].y,hull[c].x,hull[c].y);
      lines.push({a:hull[b],b:hull[c]});
    }
    else{
      c = a;
    }
    a = b;
    b = c;
    c = min(a + 1, N);
  }*/
}


function gauss_dist(){
  clear();
  points = [];
  var n = input.value();
  for (var i = 0; i < n; i++){
    let x = randomGaussian(600,70);
    let y = randomGaussian(400,70);
    let p = createVector(x,y);
    if (!points.includes(p) && (x<1200 && y < 800)){
      create_point(p);
    }
    else{
      i--;
    }
  }
}

function even_dist(){
  clear();
  points = [];
  var n = input.value();
  for (var i = 0; i < n; i++){
    let x = random(0,1200);
    let y = random(0,800);
    let p = createVector(x,y);
    if (!points.includes(p)){
      create_point(p);
    }
    else{
      i--;
    }
  }
}

function distance_compare(a,b){
  return euk_dist(a.a,a.b) - euk_dist(b.a,b.b);
}

function check_crossing(lines,triangulation){
  for(let j = 0; j < triangulation.length; j++){

    let D = (lines.b.x - lines.a.x) * (triangulation[j].b.y - triangulation[j].a.y) - (triangulation[j].b.x - triangulation[j].a.x) * (lines.b.y - lines.a.y);
    let A = (triangulation[j].b.x - triangulation[j].a.x) * (lines.a.y - triangulation[j].a.y) -  (lines.a.x - triangulation[j].a.x) * (triangulation[j].b.y - triangulation[j].a.y);
    let B = (lines.b.x - lines.a.x) * (lines.a.y - triangulation[j].a.y) - (lines.a.x - triangulation[j].a.x) * (lines.b.y - lines.a.y);

    if (equals(D, 0) && equals(A, 0) && equals(B, 0)) {
      return false; // soupadajo
    }

    if(equals(D,0)){
      continue;
    }

    var Ua = A/D;
    var Ub = B/D;

    if (equals(Ua, 1) || equals(Ua, 0) || equals(Ub, 1) || equals(Ub, 0)) {
      continue;
    }

    if(Ua > 0 && Ua < 1 && Ub > 0 && Ub < 1 ){
      return false;
    }
  
  }
  //line(lines.a.x,lines.a.y,lines.b.x,lines.b.y);
  return true;
}

function greedy_triangulation(){
  let lines = [];

  for (let i = 0; i < points.length; i++) {
    for (let j = i+1; j < points.length; j++) {
      lines.push({a:points[i], b:points[j]}); 
    }
  }

  lines.sort(distance_compare);
  triangulation.push(lines[0]);

  line(triangulation[0].a.x,triangulation[0].a.y,triangulation[0].b.x,triangulation[0].b.y);
  let n = points.length;
  grahams_scan();
  let k = hull.length;
  let i = 1;
  while(triangulation.length < 3*n-3-k){
    if(check_crossing(lines[i],triangulation)){
      line(lines[i].a.x,lines[i].a.y,lines[i].b.x,lines[i].b.y);
      triangulation.push(lines[i]);
    }
    i++;
  }

}

function draw() {

}
