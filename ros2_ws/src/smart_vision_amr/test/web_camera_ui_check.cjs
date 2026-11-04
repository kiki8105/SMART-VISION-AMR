// Exercise real camera rendering/polling without HTTP, ROS, or camera access.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const layout=source.slice(source.indexOf('  function renderCameraLayout(){'),source.indexOf('  function renderLocalization(){'));
const camera=source.slice(source.indexOf('  function updateCamera(){'),source.indexOf("  $('#map-view-plan').addEventListener"));
const elements=new Map(),listeners=new Map(),requests=[];
function element(selector){
 const classes=new Set(),attributes={};let src='';
 const item={hidden:false,textContent:'',attributes,classes,
  classList:{toggle(name,value){if(value)classes.add(name);else classes.delete(name);}},
  setAttribute(key,value){attributes[key]=value;},
  removeAttribute(key){delete attributes[key];if(key==='src')src='';},
  addEventListener(name,callback){listeners.set(selector+':'+name,callback);}};
 Object.defineProperty(item,'src',{get(){return src;},set(value){src=value;requests.push({selector,url:value});}});
 return item;
}
const $=selector=>{if(!elements.has(selector))elements.set(selector,element(selector));return elements.get(selector);};
const sandbox={$,document:{hidden:false},snapshot:null,initialized:true,disconnected:false,
 cameraLoading:false,frontCameraLoading:false};
vm.createContext(sandbox);vm.runInContext(layout+'\n'+camera+'\nglobalThis.update=updateCamera;',sandbox);
const frontRequests=()=>requests.filter(item=>item.selector==='#front-camera-image').length;
const rearRequests=()=>requests.filter(item=>item.selector==='#camera-image').length;
function loaded(selector){listeners.get(selector+':load')();}
function fixture(flag){const robot={camera_age:.1,front_camera_age:.2};if(flag!==undefined)robot.front_camera_enabled=flag;return{robot};}

sandbox.snapshot=fixture();sandbox.update();
assert.equal(frontRequests(),0,'missing flag must not poll the front JPEG');assert.equal(rearRequests(),1);
assert.equal($('#front-camera-feed').hidden,true);assert.equal($('#camera-panel-title').textContent,'후방 카메라');
assert.match($('#camera-panel').attributes['aria-label'],/후방 카메라/);
assert.match(requests[0].url,/^\/api\/camera\.jpg\?/);

// Disable while an older front request is still loading; rear continues.
loaded('#camera-image');sandbox.snapshot=fixture(false);sandbox.update();
assert.equal(frontRequests(),0,'explicit false skips front even when its age is fresh');
assert.equal(rearRequests(),2,'rear JPEG polling continues');assert.equal(sandbox.frontCameraLoading,false);
assert.equal($('#front-camera-feed').hidden,true);assert.equal($('#front-camera-image').src,'');
assert.equal($('#camera-panel-title').textContent,'후방 카메라');
assert.equal($('#camera-panel').attributes['aria-label'],'후방 카메라와 ArUco 검출');
assert.equal($('#camera-panel').classes.has('single-camera'),true);
loaded('#front-camera-image');assert.equal($('#front-camera-image').hidden,true);
listeners.get('#front-camera-image:error')();
assert.equal($('#front-camera-feed').hidden,true);assert.equal($('#front-camera-empty').hidden,true);

sandbox.snapshot=fixture(true);sandbox.update();
assert.equal(frontRequests(),1,'reenabling clears old loading state and resumes front polling');
assert.equal($('#front-camera-feed').hidden,false);assert.equal($('#camera-panel').classes.has('single-camera'),false);
assert.match(requests.find(item=>item.selector==='#front-camera-image').url,/^\/api\/camera\/front\.jpg\?/);
loaded('#front-camera-image');assert.equal($('#front-camera-image').hidden,false);
for(const flag of [undefined,true,0,'false']){
 sandbox.snapshot=fixture(flag);sandbox.frontCameraLoading=false;sandbox.update();
 assert.equal($('#front-camera-feed').hidden,flag!==true,'only explicit boolean true enables the front feed');
}
const before=requests.length;sandbox.document.hidden=true;sandbox.update();assert.equal(requests.length,before);
sandbox.document.hidden=false;sandbox.disconnected=true;sandbox.update();assert.equal(requests.length,before);
sandbox.disconnected=false;sandbox.initialized=false;sandbox.update();assert.equal(requests.length,before);
const html=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/index.html','utf8');
const css=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/styles.css','utf8');
assert.match(html,/id="front-camera-feed"[^>]*hidden/);assert.match(html,/id="camera-panel-title"/);
assert.doesNotMatch(html,/\/camera\/front\/image_raw/);
assert.match(css,/\.camera-panel\.single-camera \.camera-feed\{border-top:0\}/);
console.log('PASS: the rear-only default hides the front feed and JPEG polling, handles late events, and allows explicit front reenabling.');
