import { App } from './src/App.js';

// Browsers (Firefox notably) cache number-input values across reloads —
// edits to HTML `value="…"` defaults silently lose to the previous session.
// Force every number input back to its HTML defaultValue before App boots
// so editing the HTML actually changes what users see on reload. Saved
// tunings (localStorage-backed) survive because they're applied later via
// loadTuning(), not via input restoration.
function resetNumberInputsToHtmlDefaults() {
	for (const el of document.querySelectorAll('input[type=number]')) {
		el.value = el.defaultValue;
	}
}

// Same problem as the number inputs: Firefox (and others) restore SELECT
// values across reloads, which silently overrides the HTML's `selected`
// attribute. Force every select back to whichever <option selected> is
// declared in the HTML so editing the markup actually changes what the
// user sees on reload.
function resetSelectsToHtmlDefaults() {
	for (const sel of document.querySelectorAll('select')) {
		const def = sel.querySelector('option[selected]');
		if (def) sel.value = def.value;
	}
}

window.addEventListener('DOMContentLoaded', () => {
	resetNumberInputsToHtmlDefaults();
	resetSelectsToHtmlDefaults();
	window.app = new App();
});
