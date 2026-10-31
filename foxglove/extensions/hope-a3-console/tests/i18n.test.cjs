const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const ts = require('typescript');
const React = require('react');
const { renderToStaticMarkup } = require('react-dom/server');
const load = require('./loadSource.cjs');
const { en } = load('messages.en');
const { zh } = load('messages.zh');
const { createTranslator, message, LocalizedError, errorText, readLocale } = load('i18n');
const { LanguageContext } = load('LanguageContext');
const han = /[\u3400-\u9fff]/;
const placeholders = text => [...text.matchAll(/\{(\w+)\}/g)].map(m=>m[1]).sort();

test('catalogs are complete, English-only, and have matching interpolation contracts', () => {
 assert.deepEqual(Object.keys(zh).sort(), Object.keys(en).sort());
 for(const key of Object.keys(en)) {
   assert.ok(en[key].trim() && zh[key].trim(), key);
   assert.doesNotMatch(en[key], han, key);
   // Technical terms can stay English; explanatory prose in Chinese mode cannot.
   if (zh[key].length > 100) assert.match(zh[key], han, key+' untranslated paragraph');
   assert.doesNotMatch(zh[key], /(?:[A-Za-z]+[ ,;:.-]+){8}[A-Za-z]+/, key+' untranslated prose');
   assert.deepEqual(placeholders(en[key]), placeholders(zh[key]), key);
   const params=Object.fromEntries(placeholders(en[key]).map(p=>[p,'sample']));
   assert.doesNotMatch(createTranslator('en')(key,params), /\{\w+\}/, key);
 }
});

test('nested notices and errors remain language-neutral until rendered; independent panels do not leak locale', () => {
 const english=createTranslator('en'), chinese=createTranslator('zh');
 const value=message('hopeA3Console.failed', {error:message('robotConnection.enterAValidLocalIpv4AddressForThe')});
 const error=new LocalizedError(value);
 assert.match(chinese.text(errorText(error)),/请输入/);
 assert.match(english.text(errorText(error)),/Enter a valid local IPv4/);
 assert.match(chinese.text(errorText(error)),/请输入/);
 assert.equal(english.text(errorText(new Error('backend_code=12'))),'backend_code=12');
 assert.equal(readLocale({language:'en'}),'en');
 for(const state of [undefined,{}, {language:'unexpected'}]) assert.equal(readLocale(state),'zh');
});

test('actual English React controls render localized labels and retain protocol values', () => {
 const render = (Component, props, locale='en') => renderToStaticMarkup(React.createElement(LanguageContext.Provider,
   {value:createTranslator(locale)},React.createElement(Component,props)));
 const {PlaneModeControls}=load('PlaneModeControls');
 const html=render(PlaneModeControls,{available:true,state:{requested:'stable_per_ball',lastCommand:'fixed',flight:2,
   supported:true,stable:false},onApply:()=>{throw new Error('must not run')}});
 assert.doesNotMatch(html,han);
 assert.match(html,/Update after posture settles/);
 for(const mode of ['fixed','per_ball','stable_per_ball']) assert.ok(html.includes(`value="${mode}"`));
 const {BallYFilterControls}=load('BallYFilterControls');
 const filter=render(BallYFilterControls,{available:false,kernelMode:false,onApply:()=>{},onPreview:()=>{},
   initialState:{ballYFilterDraft:{enabled:false,min:'-1.3',max:'-0.55'},ballYFilterDraftEdited:true}});
 assert.doesNotMatch(filter,han);assert.match(filter,/active state unknown/);assert.doesNotMatch(filter,/checked=""/);
 const {HitSpeedScaleControls}=load('HitSpeedScaleControls');
 assert.doesNotMatch(render(HitSpeedScaleControls,{available:true,kernelMode:false,state:{scale:1,persistent:true},onApply:()=>{}}),han);
 const {HopeA3Console}=load('HopeA3Console','\nexport { HopeA3Console };');
 const main=renderToStaticMarkup(React.createElement(HopeA3Console,{context:{initialState:{language:'en'}}}));
 assert.doesNotMatch(main,han);
 assert.match(main,/class="language-switch"/);
 assert.match(main,/aria-pressed="true">English/);
 assert.match(main,/Connection target IP/);
 const chinese=renderToStaticMarkup(React.createElement(HopeA3Console,{context:{initialState:{language:'zh'}}}));
 assert.match(chinese,/每个发球策略会选择各自匹配的 CSV/);
 assert.match(chinese,/左摇杆控制前后和左右移动/);
 assert.match(chinese,/上传会使用 Runner 的 CSV 校验器/);
 assert.doesNotMatch(chinese,/Wait for ACTIVE, center the sticks|Switch while STOPPED, then START/);
});

test('new UI text belongs in catalogs, and translations never become comparison operands', () => {
 const root=path.join(__dirname,'../src');
 for(const file of fs.readdirSync(root).filter(f=>/\.tsx?$/.test(f)&&!f.startsWith('messages.'))) {
   const source=fs.readFileSync(path.join(root,file),'utf8');
   const ast=ts.createSourceFile(file,source,ts.ScriptTarget.Latest,true);
   function visit(node) {
     const location=file+':'+(ast.getLineAndCharacterOfPosition(node.getStart(ast)).line+1);
     if(ts.isStringLiteral(node)||ts.isNoSubstitutionTemplateLiteral(node)||ts.isJsxText(node)) {
       assert.doesNotMatch(node.text,han,location+' put translations in messages.*.ts');
       const text=node.text.trim();
       if(ts.isJsxText(node)) assert.ok(!/[A-Za-z]/.test(text),location+' raw JSX text');
       // Technical code/metadata and CSS classes are intentionally not translated.
       const technical=file==='index.ts'||file==='mocapSetup.ts'||text.startsWith('<')||text.startsWith('@')||
         node.parent.getText(ast).startsWith('className=')||ts.isImportDeclaration(node.parent);
       if(!technical) assert.ok(!/[A-Za-z]+\s+[A-Za-z]/.test(text),location+' raw prose: '+text);
     }
     if(ts.isBinaryExpression(node)&&[ts.SyntaxKind.EqualsEqualsEqualsToken,ts.SyntaxKind.ExclamationEqualsEqualsToken].includes(node.operatorToken.kind)) {
       for(const operand of [node.left,node.right]) assert.ok(!(ts.isCallExpression(operand)&&operand.expression.getText(ast)==='t'),location+' translated protocol comparison');
     }
     ts.forEachChild(node,visit);
   }
   visit(ast);
 }
});
