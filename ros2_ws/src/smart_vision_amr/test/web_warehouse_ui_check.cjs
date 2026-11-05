// Execute the actual dynamic selectors and heading function without ROS/browser.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const inventory=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/inventory.js','utf8');
const markers=Array.from({length:11},(_,id)=>({id,name:id?`Sector ${id}`:'홈',x:id,y:1,yaw:0,camera:'rear'}));
const elements=new Map();
function element(){return {children:[],dataset:{},value:'',handlers:{},
  replaceChildren(){this.children=[];},append(...children){this.children.push(...children);},
  setAttribute(){},addEventListener(event,callback){this.handlers[event]=callback;}};}
const $=selector=>{if(!elements.has(selector))elements.set(selector,element());return elements.get(selector);};
const sandbox={$ ,document:{createElement:element},snapshot:{markers},draft:[],dirty:false,
  markerKey:'',markerLabel:value=>typeof value==='object'?value.name:markers.find(m=>m.id===value).name,
  delivery:()=>({active:false}),notify:()=>{},same:(a,b)=>JSON.stringify(a)===JSON.stringify(b),
  savedRoute:()=>[],renderEditor:()=>{},renderControls:()=>{},drawMap:()=>{}};
vm.createContext(sandbox);
const buttons=source.slice(source.indexOf('  function renderMarkerButtons(){'),source.indexOf('  function renderControls(){'));
const heading=source.slice(source.indexOf('  function markerScreenHeadingDegrees('),source.indexOf('  function drawMap(){'));
const options=inventory.slice(inventory.indexOf('    const markers=data.markers||[],key=JSON.stringify(markers);'),
  inventory.indexOf('    const requests=data.requests||[],nextHistoryKey='));
vm.runInContext(buttons+heading+'\nglobalThis.buttons=renderMarkerButtons;globalThis.heading=markerScreenHeadingDegrees;'+
  '\nglobalThis.options=function(data){'+options+'};',sandbox);
sandbox.buttons();
assert.deepEqual($('#add-markers').children.map(item=>item.textContent),markers.map(m=>m.name));
$('#add-markers').children[10].handlers.click();
assert.equal(sandbox.draft[0],10,'Sector 10 is selectable for an operator route');
sandbox.options({markers});
assert.deepEqual($('#marker-id').children.slice(1).map(item=>item.value),markers.map(m=>String(m.id)),
  'inventory options include every configured sector, including Sector 10');
$('#marker-id').value='10';
sandbox.options({markers:markers.map(m=>({...m,name:m.name+' updated'}))});
assert.equal($('#marker-id').value,'10','a refreshed ten-sector list preserves the selected Sector 10');
assert.equal(sandbox.heading({id:3,yaw:0},0,0),0,'warehouse Sector 3 follows its actual yaw');
assert.equal(sandbox.heading({id:3,yaw:0,screen_heading_deg:90},0,0),90,
  'the classroom presentation-only override remains explicit');
assert.equal(markers[3].yaw,0,'UI overrides never mutate a navigation target');
console.log('Warehouse dynamic sector selectors and map headings PASS');
